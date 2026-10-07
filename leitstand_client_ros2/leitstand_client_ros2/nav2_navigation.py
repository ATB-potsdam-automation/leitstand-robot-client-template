"""Navigation against Nav2 + robot_localization, the reference implementation of ``Navigation``.

Runs stages as ``NavigateThroughPoses`` goals and coverage stages as one ``FollowPath`` goal
over the planned route. WGS84 waypoints are projected into the map frame through
``navsat_transform_node``'s ``/fromLL`` service, once per stage, so the projection stays
consistent with the global EKF datum.

Frame transitions (``map_server.load_map`` to swap occupancy grids, ``SetDatum`` to re-anchor
on a Site, ``/initialpose`` to re-init localization) are not implemented: stages that demand a
frame switch fail with a structured error.

Threading: rclpy futures are bridged into asyncio with ``_await_rclpy``; the node is spun by
the caller, not here.
"""

from __future__ import annotations

import asyncio
import contextlib
import functools
import logging
import math
import threading
import time
import uuid
from collections.abc import AsyncIterator
from typing import Any, Callable

from nav_msgs.msg import Path

from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client import geo, proto_json
from leitstand_client.navigation import CancelRequested, StageResult, is_immediate
from leitstand_client_ros2.config import Nav2Config

logger = logging.getLogger(__name__)


async def _await_rclpy(future: Any) -> Any:
    """Await an rclpy.task.Future from the asyncio event loop.

    asyncio.wrap_future only accepts concurrent.futures.Future; rclpy uses its
    own Future type. We replicate what wrap_future does internally: create an
    asyncio.Future, wire a done-callback via call_soon_threadsafe, and await it.
    The done-guard (aio_fut.done()) makes the callback a no-op if asyncio.wait_for
    already cancelled the awaiting coroutine.
    """
    loop = asyncio.get_running_loop()
    aio_fut: asyncio.Future[Any] = loop.create_future()

    def _on_done(_: Any) -> None:
        if not aio_fut.done():
            loop.call_soon_threadsafe(aio_fut.set_result, None)

    future.add_done_callback(_on_done)
    await aio_fut
    return future.result()


# Time the controller gets to stop on its own before zero velocity overrides its deceleration.
_STOP_GRACE_S = 1.0
_UNCONFIRMED_STOP = "The machine did not confirm that it stopped."


def _cancel_if_accepted_late(future: Any) -> None:
    """Cancel a goal whose acceptance arrived after we stopped waiting for it."""
    try:
        handle = future.result()
    except Exception:  # noqa: BLE001 - a goal that failed to send runs nothing
        return
    if handle is not None and handle.accepted:
        handle.cancel_goal_async()


def _release(release: Callable[..., Any], *args: Any) -> None:
    """Call one release step, logging a failure so the remaining steps still run."""
    try:
        release(*args)
    except Exception as exc:  # noqa: BLE001
        name = getattr(release, "__qualname__", repr(release))
        logger.warning("[nav2_navigation] release %s failed: %s", name, exc)


def _with_unconfirmed_stop(result: StageResult) -> StageResult:
    """Return the result with a note that the machine may still be moving."""
    error = mission_state_pb2.Error()
    if result.error is None:
        error.type = "stop_unconfirmed"
        error.description = _UNCONFIRMED_STOP
    else:
        error.CopyFrom(result.error)
        error.description = f"{error.description} {_UNCONFIRMED_STOP}".strip()
    # A machine that may still be moving is always a fatal error.
    error.severity = mission_state_pb2.ERROR_SEVERITY_FATAL
    error.references.append(mission_state_pb2.ErrorReference(key="stop_confirmed", value="false"))
    return StageResult(status=result.status, error=error)


# All ROS imports are deferred so the module can be imported in environments
# without a sourced ROS distro; construction is what fails when ROS is absent.


def _filled(
    points: list[tuple[float, float, float | None]], spacing_m: float
) -> list[tuple[float, float, float | None]]:
    """Return the route with any run longer than ``spacing_m`` filled in at that interval.

    A straight is exactly recoverable from its endpoints, so interpolating adds no information and
    loses none; a turn arrives already dense and passes through untouched. The heading carried is
    the one the run is driven at, which for a straight does not change along it.
    """
    out: list[tuple[float, float, float | None]] = []
    for point in points:
        if out:
            start = out[-1]
            span = math.dist((start[0], start[1]), (point[0], point[1]))
            for step in range(1, int(span / spacing_m) + 1):
                fraction = step * spacing_m / span
                if fraction >= 1.0:
                    break
                out.append(
                    (
                        start[0] + (point[0] - start[0]) * fraction,
                        start[1] + (point[1] - start[1]) * fraction,
                        start[2],
                    )
                )
        out.append(point)
    return out


def _xy(pose: Any) -> tuple[float, float]:
    return pose.pose.position.x, pose.pose.position.y


def _backed_off(poses: list[Any], index: int, distance_m: float) -> int:
    """Return the index at least ``distance_m`` of route behind ``index``, or 0 if none is."""
    remaining = distance_m
    while index > 0 and remaining > 0.0:
        remaining -= math.dist(_xy(poses[index - 1]), _xy(poses[index]))
        index -= 1
    return index


class Nav2Navigation:
    """Executes stages against Nav2 via rclpy, on a node the caller owns and spins."""

    def __init__(self, config: Nav2Config, node: Any) -> None:
        # ROS is imported at construction, not at module load, so the path logic in this module
        # is unit-tested without a ROS installation.
        import rclpy
        import tf2_ros
        from action_msgs.msg import GoalStatus, GoalStatusArray
        from action_msgs.srv import CancelGoal
        from geographic_msgs.msg import GeoPoint
        from geometry_msgs.msg import PoseStamped, Twist
        from nav2_msgs.action import FollowPath, NavigateThroughPoses, NavigateToPose
        from nav_msgs.msg import Path
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_action_status_default
        from robot_localization.srv import FromLL
        from unique_identifier_msgs.msg import UUID

        self._cfg = config
        self._node = node
        self._rclpy = rclpy
        self._GeoPoint = GeoPoint
        self._FromLL = FromLL
        self._GoalStatus = GoalStatus
        # A goal in one of these may still move the machine.
        self._active_statuses = frozenset(
            {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING}
        )
        # UNKNOWN is not among them, so a goal whose state nobody knows is never taken as ended.
        self._ended_statuses = frozenset(
            {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
        )
        self._Twist = Twist
        self._Path = Path
        self._PoseStamped = PoseStamped
        self._NavigateThroughPoses = NavigateThroughPoses
        self._NavigateToPose = NavigateToPose
        self._FollowPath = FollowPath
        self._from_ll_timeout_s = config.projection_timeout_s
        self._send_goal_timeout_s = config.send_goal_timeout_s

        self._action_client = ActionClient(
            node, NavigateThroughPoses, config.actions.navigate_through_poses
        )
        self._approach_client = ActionClient(node, NavigateToPose, config.actions.navigate_to_pose)
        self._follow_path_client = ActionClient(node, FollowPath, config.actions.follow_path)
        self._CancelGoal = CancelGoal
        # The goal is stopping or had already ended.
        self._cancel_accepted = frozenset(
            {CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_GOAL_TERMINATED}
        )
        action_names = (
            config.actions.navigate_through_poses,
            config.actions.navigate_to_pose,
            config.actions.follow_path,
        )
        self._cancel_all_clients = [
            node.create_client(CancelGoal, f"{name}/_action/cancel_goal") for name in action_names
        ]
        self._UUID = UUID
        # Per action server, each goal's latest status by goal id. Written by the rclpy executor
        # thread, read by the asyncio loop, both under the lock.
        self._goal_statuses: dict[str, dict[bytes, int]] = {}
        self._goal_statuses_lock = threading.Lock()
        self._status_subs = [
            node.create_subscription(
                GoalStatusArray,
                f"{name}/_action/status",
                functools.partial(self._store_goal_statuses, name),
                qos_profile_action_status_default,
            )
            for name in action_names
        ]
        # Goals this stage asked for that no status has shown ended yet. Guarded by the lock.
        self._requested_goals: set[bytes] = set()
        self._from_ll_client = node.create_client(FromLL, config.projection_service)
        # Nav2 stores a FollowPath goal and never republishes it, so RViz would otherwise show only
        # the controller's window; latched so a late subscriber still sees the commanded path.
        self._path_pub = node.create_publisher(
            Path,
            config.commanded_path_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._stop_pub = (
            node.create_publisher(Twist, config.stop_topic, 10) if config.stop_topic else None
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._running = threading.Event()
        self._running.set()
        # Set once a pause has brought the machine to a stop, cleared on resume.
        self._paused = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Release what this object created on the node; the node itself belongs to the caller."""
        _release(self._tf_listener.unregister)
        for action_client in (self._action_client, self._approach_client, self._follow_path_client):
            _release(action_client.destroy)
        for client in (self._from_ll_client, *self._cancel_all_clients):
            _release(self._node.destroy_client, client)
        for subscription in self._status_subs:
            _release(self._node.destroy_subscription, subscription)
        for publisher in (self._path_pub, self._stop_pub):
            if publisher is not None:
                _release(self._node.destroy_publisher, publisher)

    # ------------------------------------------------------------------
    # Pause / resume
    # ------------------------------------------------------------------

    def request_pause(self) -> None:
        self._running.clear()
        logger.info("[nav2_navigation] pause requested")

    def request_resume(self) -> None:
        self._running.set()
        logger.info("[nav2_navigation] resume requested")

    def is_paused(self) -> bool:
        """True once the goal cancel has returned and the machine stands still."""
        return self._paused.is_set()

    # ------------------------------------------------------------------
    # Dispatch preflight
    # ------------------------------------------------------------------

    def check_ready(self, mission: mission_pb2.Mission, timeout_s: float = 1.0) -> str | None:
        """Probe mission-wide readiness; return None if ready else a reason.

        Only the action server is probed: ``/fromLL`` may be unavailable at dispatch time (an
        indoor first stage before GPS is acquired), so it stays a per-stage check.
        """
        if not self._action_client.wait_for_server(timeout_sec=timeout_s):
            return "navigation system not ready (navigate_through_poses action server unavailable)"
        return None

    # ------------------------------------------------------------------
    # Stage execution
    # ------------------------------------------------------------------

    async def execute_stage(
        self,
        stage: mission_pb2.Stage,
        robot_id: str,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None = None,
    ) -> StageResult:
        # A stage that ends early can leave a goal running that nothing watches any more, also when a
        # cancel arrived in the same moment as a navigation failure.
        with self._goal_statuses_lock:
            self._requested_goals = set()
        try:
            result = await self._execute_stage(stage, robot_id, cancel_requested, on_progress)
        # A stage task cancelled from outside skips this, which only happens at shutdown.
        except Exception:
            if not await self._ensure_stopped():
                logger.warning("[nav2_navigation] stage raised and its stop is unconfirmed")
            raise
        if result.status != mission_state_pb2.STAGE_STATUS_FINISHED:
            if not await self._ensure_stopped():
                result = _with_unconfirmed_stop(result)
        return result

    async def _execute_stage(
        self,
        stage: mission_pb2.Stage,
        robot_id: str,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None = None,
    ) -> StageResult:
        if cancel_requested() is not None:
            return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)

        # The planned line is followed as one path, because planning between its points would
        # reach the same waypoints by a route nobody approved and leave the ground beside it unworked.
        if stage.kind == mission_pb2.STAGE_KIND_COVERAGE and stage.coverage.segments:
            return await self._follow_planned_path(stage, cancel_requested, on_progress)

        try:
            poses = await self._build_poses(stage)
            path=Path()
            path.header.stamp = self._node.get_clock().now().to_msg()
            path.header.frame_id = self._cfg.frames.map
            path.poses=poses
            self._path_pub.publish(path)

        except _FrameSwitchUnsupported as exc:
            return StageResult(
                status=mission_state_pb2.STAGE_STATUS_FAILED,
                error=mission_state_pb2.Error(
                    severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                    type="frame_switch_unsupported",
                    description=str(exc),
                ),
            )
        except _FromLLFailed as exc:
            return StageResult(
                status=mission_state_pb2.STAGE_STATUS_FAILED,
                error=mission_state_pb2.Error(
                    severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                    type="wgs84_projection_failed",
                    description=str(exc),
                ),
            )

        goal = self._NavigateThroughPoses.Goal()
        goal.poses = poses

        if not self._action_client.wait_for_server(timeout_sec=self._send_goal_timeout_s):
            return StageResult(
                status=mission_state_pb2.STAGE_STATUS_FAILED,
                error=mission_state_pb2.Error(
                    severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                    type="nav2_unavailable",
                    description="navigate_through_poses action server did not appear",
                ),
            )

        # Build a feedback callback that reports fractional progress via on_progress.
        # NavigateThroughPoses.Feedback carries number_of_poses_remaining (confirmed
        # against installed nav2_msgs). Called on the rclpy executor thread; on_progress
        # must marshal onto the asyncio loop (the caller's responsibility).
        total = len(poses)
        last_progress = [0.0]
        # Nav2's count of the waypoints still ahead, which a resume continues from. Best effort:
        # the default behaviour tree updates it every few seconds, and a tree without
        # RemovePassedGoals never lowers it.
        remaining = [total]

        def _fb(fb_msg: Any) -> None:
            rem = getattr(fb_msg.feedback, "number_of_poses_remaining", None)
            if rem is None or total <= 0:
                return
            remaining[0] = rem
            if on_progress is not None:
                last_progress[0] = max(last_progress[0], min(1.0, 1.0 - rem / total))
                on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, last_progress[0])

        try:
            goal_handle = await self._send_goal(self._action_client, goal, _fb)
        except asyncio.TimeoutError:
            return self._send_goal_timed_out()
        if goal_handle is None or not goal_handle.accepted:
            return StageResult(
                status=mission_state_pb2.STAGE_STATUS_FAILED,
                error=mission_state_pb2.Error(
                    severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                    type="nav2_goal_rejected",
                    description="Nav2 rejected the goal",
                ),
            )

        # Goal accepted: signal RUNNING at progress 0 so the consumer sees the
        # transition immediately (before the first feedback arrives).
        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, 0.0)

        while True:
            outcome, detail = await self._wait_goal(
                goal_handle, cancel_requested, on_progress, lambda: last_progress[0]
            )
            if outcome == "abandoned":
                return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED, error=detail)
            if outcome == "done":
                break
            # Resending every waypoint would drive the machine back to the first one.
            goal.poses = poses[min(total - remaining[0], total - 1) :]
            try:
                goal_handle = await self._send_goal(self._action_client, goal, _fb)
            except asyncio.TimeoutError:
                return self._send_goal_timed_out()
            if goal_handle is None or not goal_handle.accepted:
                return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)

        # rclpy returns a wrapper with .status and .result. SUCCEEDED == 4 per
        # action_msgs/GoalStatus.
        status_code = getattr(detail, "status", None)
        if status_code == 4:
            return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)
        return StageResult(
            status=mission_state_pb2.STAGE_STATUS_FAILED,
            error=mission_state_pb2.Error(
                severity=mission_state_pb2.ERROR_SEVERITY_WARNING,
                type="nav2_terminal_failure",
                description=f"Nav2 terminal status {status_code}",
                references=[
                    mission_state_pb2.ErrorReference(key="status_code", value=str(status_code))
                ],
            ),
        )

    # ------------------------------------------------------------------
    # Planned-path execution
    # ------------------------------------------------------------------

    async def _follow_planned_path(
        self,
        stage: mission_pb2.Stage,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None,
    ) -> StageResult:
        """Drive a coverage stage's planned path as one FollowPath goal.

        One goal for the whole stage rather than one per swath: a goal ends at its goal checker, so
        per-swath goals would stop the machine at both ends of every pass.
        """
        try:
            poses = await self._build_path_poses(stage)
        except _FromLLFailed as exc:
            return StageResult(
                status=mission_state_pb2.STAGE_STATUS_FAILED,
                error=mission_state_pb2.Error(
                    severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                    type="wgs84_projection_failed",
                    description=str(exc),
                ),
            )
        except _FrameSwitchUnsupported as exc:
            return StageResult(
                status=mission_state_pb2.STAGE_STATUS_FAILED,
                error=mission_state_pb2.Error(
                    severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                    type="frame_switch_unsupported",
                    description=str(exc),
                ),
            )

        if not self._follow_path_client.wait_for_server(timeout_sec=self._send_goal_timeout_s):
            return StageResult(
                status=mission_state_pb2.STAGE_STATUS_FAILED,
                error=mission_state_pb2.Error(
                    severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                    type="nav2_unavailable",
                    description="follow_path action server did not appear",
                ),
            )

        start = 0
        begun = False
        aborts = 0
        while aborts < self._cfg.follow_path_attempts:
            if cancel_requested() is not None:
                return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)

            # A plan is entered at its head and re-entered where the machine stands, a little way
            # back so the controller has path behind it to match against. Nearest-first on entry
            # would hand a machine parked beside the last swath a finished field.
            target = 0
            if begun:
                nearest = self._nearest_index(poses)
                if nearest is None:
                    return StageResult(
                        status=mission_state_pb2.STAGE_STATUS_FAILED,
                        error=mission_state_pb2.Error(
                            severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                            type="resume_position_unknown",
                            description=(
                                "cannot locate the machine on the path: map to base_footprint "
                                "is unavailable"
                            ),
                        ),
                    )
                target = nearest
                start = _backed_off(poses, nearest, self._cfg.resume_behind_m)
                logger.info("[nav2_navigation] resuming at %d of %d", start, len(poses))

            # The controller follows a path only where it crosses the local costmap, so a machine
            # standing off the path is driven onto it first, planning around whatever lies between.
            if not self._within(poses[target], self._cfg.approach_tolerance_m):
                logger.info(
                    "[nav2_navigation] driving to pose %d of %d before following",
                    target,
                    len(poses),
                )
                if on_progress is not None:
                    on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, start / max(len(poses), 1))
                result, aborted = await self._drive_to(poses[target], cancel_requested)
                if result is not None:
                    return result
                if aborted:
                    aborts += 1
                    logger.warning(
                        "[nav2_navigation] approach aborted (%d/%d)",
                        aborts,
                        self._cfg.follow_path_attempts,
                    )
                    await asyncio.sleep(self._cfg.follow_path_retry_s)
                    continue

            result, aborted = await self._run_follow_path(
                poses, start, cancel_requested, on_progress
            )
            if result is not None:
                return result
            begun = True
            if aborted:
                # A pause is the operator's doing and must not spend the budget that exists for a
                # controller that cannot make progress.
                aborts += 1
                logger.warning(
                    "[nav2_navigation] follow_path aborted (%d/%d)",
                    aborts,
                    self._cfg.follow_path_attempts,
                )
                await asyncio.sleep(self._cfg.follow_path_retry_s)

        return StageResult(
            status=mission_state_pb2.STAGE_STATUS_FAILED,
            error=mission_state_pb2.Error(
                severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                type="follow_path_exhausted",
                description=f"follow_path aborted {self._cfg.follow_path_attempts} times",
            ),
        )

    async def _run_follow_path(
        self,
        poses: list[Any],
        start: int,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None,
    ) -> tuple[StageResult | None, bool]:
        """Drive ``poses[start:]`` once.

        Returns the stage's result when it settles, or ``None`` to be re-issued, paired with
        whether that re-issue follows an abort rather than a pause.
        """
        path = self._Path()
        path.header.stamp = self._node.get_clock().now().to_msg()
        path.header.frame_id = self._cfg.frames.map
        path.poses = poses[start:]

        goal = self._FollowPath.Goal()
        goal.path = path
        goal.controller_id = self._cfg.controller_id
        self._path_pub.publish(path)

        try:
            handle = await self._send_goal(self._follow_path_client, goal)
        except asyncio.TimeoutError:
            return self._send_goal_timed_out(), False
        if handle is None or not handle.accepted:
            return (
                StageResult(
                    status=mission_state_pb2.STAGE_STATUS_FAILED,
                    error=mission_state_pb2.Error(
                        severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                        type="nav2_goal_rejected",
                        description="Nav2 rejected the path",
                    ),
                ),
                False,
            )

        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, start / max(len(poses), 1))

        last_progress = [start / max(len(poses), 1)]

        def _tick() -> None:
            # Position comes from TF rather than from the action's distance_to_goal, whose meaning
            # is unconfirmed: on a boustrophedon the finish lies beside the start. A momentary gap
            # in TF leaves the last figure standing rather than reporting a made-up one.
            reached = self._nearest_index(poses)
            if on_progress is not None and reached is not None:
                last_progress[0] = reached / max(len(poses), 1)
                on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, last_progress[0])

        outcome, detail = await self._wait_goal(
            handle,
            cancel_requested,
            on_progress,
            lambda: last_progress[0],
            on_tick=_tick,
            tick_s=self._cfg.progress_tick_s,
        )
        if outcome == "abandoned":
            return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED, error=detail), False
        if outcome == "paused":
            return None, False
        if getattr(detail, "status", None) == self._GoalStatus.STATUS_SUCCEEDED:
            return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED), False
        return None, True

    async def _drive_to(
        self, pose: Any, cancel_requested: CancelRequested
    ) -> tuple[StageResult | None, bool]:
        """Navigate to one pose, planning around whatever lies on the way.

        Arrival is the navigator's, goal checker included: the heading it settles is the heading
        the path about to be followed starts from, and a controller started at an angle to its
        path on sparse poses swings between turning and driving without ever getting on it.

        Returns ``(None, False)`` on arrival, ``(None, True)`` when the machine ended up nowhere
        near the pose, or the stage's result paired with ``False`` when the stage must end here.
        """
        if not self._approach_client.wait_for_server(timeout_sec=self._send_goal_timeout_s):
            return (
                StageResult(
                    status=mission_state_pb2.STAGE_STATUS_FAILED,
                    error=mission_state_pb2.Error(
                        severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                        type="nav2_unavailable",
                        description="navigate_to_pose action server did not appear",
                    ),
                ),
                False,
            )

        goal = self._NavigateToPose.Goal()
        goal.pose = pose
        while True:
            try:
                handle = await self._send_goal(self._approach_client, goal)
            except asyncio.TimeoutError:
                return self._send_goal_timed_out(), False
            if handle is None or not handle.accepted:
                return (
                    StageResult(
                        status=mission_state_pb2.STAGE_STATUS_FAILED,
                        error=mission_state_pb2.Error(
                            severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                            type="nav2_goal_rejected",
                            description="Nav2 rejected the approach goal",
                        ),
                    ),
                    False,
                )

            outcome, detail = await self._wait_goal(
                handle, cancel_requested, None, lambda: 0.0, tick_s=self._cfg.progress_tick_s
            )
            if outcome == "abandoned":
                return StageResult(
                    status=mission_state_pb2.STAGE_STATUS_FAILED, error=detail
                ), False
            if outcome == "paused":
                continue
            if getattr(detail, "status", None) == self._GoalStatus.STATUS_SUCCEEDED:
                return None, False
            # The navigator gave up. Where it nonetheless left the machine close enough, the path
            # is reachable and the leg has done its job.
            return None, not self._within(pose, self._cfg.approach_tolerance_m)

    async def _wait_goal(
        self,
        handle: Any,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None,
        progress: Callable[[], float],
        *,
        on_tick: Callable[[], None] | None = None,
        tick_s: float = 0.1,
    ) -> tuple[str, Any]:
        """Wait on an accepted goal and return how it ended.

        ("done", result) when Nav2 finished it. A pause cancels the goal, reports PAUSED once Nav2
        reports the goal ended, holds until resume, reports RUNNING and returns ("paused", None) so
        the caller re-issues what is left. ("abandoned", error) ends the stage: a cancel has no
        error, a pause whose goal Nav2 did not confirm as ended has one.
        """
        result_task = asyncio.create_task(_await_rclpy(handle.get_result_async()))
        while not result_task.done():
            if cancel_requested() is not None:
                await self._abandon(handle, result_task, cancel_requested())
                return "abandoned", None
            if not self._running.is_set():
                acknowledged = await self._abandon(handle, result_task)
                # PAUSED promises that the machine stands still, so it waits for the goal's end.
                if not (acknowledged and await self._goals_end_within(self._send_goal_timeout_s)):
                    return "abandoned", mission_state_pb2.Error(
                        severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                        type="pause_failed",
                        description="Nav2 did not confirm that the paused goal ended.",
                    )
                self._paused.set()
                if on_progress is not None:
                    on_progress(mission_state_pb2.STAGE_STATUS_PAUSED, progress())
                while not self._running.is_set():
                    if cancel_requested() is not None:
                        self._paused.clear()
                        return "abandoned", None
                    await asyncio.sleep(0.1)
                self._paused.clear()
                if on_progress is not None:
                    on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, progress())
                return "paused", None
            if on_tick is not None:
                on_tick()
            await asyncio.sleep(tick_s)
        return "done", result_task.result()

    async def _abandon(
        self, handle: Any, result_task: asyncio.Task, mode: int | None = None
    ) -> bool:
        """Cancel a running goal, stop waiting on its result, and return whether Nav2 accepted.

        Cancelling lets the controller and the velocity smoother bring the machine to a stop. An
        IMMEDIATE cancel additionally holds zero velocity on the stop topic, if one is configured,
        until the goal has ended, at most twice the send-goal timeout: a single zero would be
        overwritten by the next controller command, and only a top-priority mux input wins over the
        controller.
        """
        holding = is_immediate(mode) and self._stop_pub is not None
        acknowledged = True
        async with self._zero_hold(holding):
            try:
                response = await asyncio.wait_for(
                    _await_rclpy(handle.cancel_goal_async()), timeout=self._send_goal_timeout_s
                )
                acknowledged = response.return_code in self._cancel_accepted
            except asyncio.TimeoutError:
                acknowledged = False
            if holding and self._rclpy.ok():
                # The zero command must outlast the goal, or the next controller command replaces it.
                try:
                    await asyncio.wait_for(
                        asyncio.shield(result_task), timeout=self._send_goal_timeout_s
                    )
                except Exception:  # noqa: BLE001 - only the waiting matters, not the result
                    pass
            result_task.cancel()
            try:
                await result_task
            except (asyncio.CancelledError, Exception):  # noqa: B014
                pass
        return acknowledged

    @contextlib.asynccontextmanager
    async def _zero_hold(self, active: bool) -> AsyncIterator[None]:
        """Hold zero velocity on the stop topic while the block runs, if active and configured."""
        if not active or self._stop_pub is None:
            yield
            return
        hold = asyncio.create_task(self._hold_zero_velocity())
        try:
            yield
        finally:
            # A hold that outlived its block would keep the machine stopped until the process died.
            hold.cancel()
            try:
                await hold
            except asyncio.CancelledError:
                pass
            except Exception:  # noqa: BLE001 - must not replace what the block raised
                logger.exception("[nav2_navigation] zero velocity hold failed")

    def _send_goal_timed_out(self) -> StageResult:
        return StageResult(
            status=mission_state_pb2.STAGE_STATUS_FAILED,
            error=mission_state_pb2.Error(
                severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                type="nav2_send_goal_timeout",
                description=f"send_goal timed out after {self._send_goal_timeout_s}s",
            ),
        )

    async def _hold_zero_velocity(self) -> None:
        zero = self._Twist()
        while True:
            self._stop_pub.publish(zero)
            await asyncio.sleep(0.05)

    async def _send_goal(self, client: Any, goal: Any, feedback_callback: Any = None) -> Any:
        """Send a goal and wait for its acceptance, bounded by the send-goal timeout.

        The goal is tracked by its id until a status shows it ended or Nav2 rejects it. A goal
        accepted after the wait is cancelled when its acceptance arrives.
        """
        goal_id = uuid.uuid4().bytes
        with self._goal_statuses_lock:
            self._requested_goals.add(goal_id)
        future = client.send_goal_async(
            goal, feedback_callback=feedback_callback, goal_uuid=self._UUID(uuid=list(goal_id))
        )
        try:
            handle = await asyncio.wait_for(_await_rclpy(future), timeout=self._send_goal_timeout_s)
        except asyncio.TimeoutError:
            future.add_done_callback(_cancel_if_accepted_late)
            raise
        if handle is None or not handle.accepted:
            with self._goal_statuses_lock:
                self._requested_goals.discard(goal_id)
        return handle

    def _store_goal_statuses(self, action: str, msg: Any) -> None:
        statuses = {bytes(s.goal_info.goal_id.uuid): s.status for s in msg.status_list}
        ended = {g for g, status in statuses.items() if status in self._ended_statuses}
        with self._goal_statuses_lock:
            self._goal_statuses[action] = statuses
            self._requested_goals -= ended

    def _any_goal_active(self) -> bool:
        """Return whether any goal may still move the machine, ours or another client's.

        A goal we asked for counts until a status shows it ended, so a missing status or an
        acceptance that has not arrived yet never reads as stopped.
        """
        with self._goal_statuses_lock:
            if self._requested_goals:
                return True
            return any(
                status in self._active_statuses
                for statuses in self._goal_statuses.values()
                for status in statuses.values()
            )

    async def _ensure_stopped(self) -> bool:
        """Cancel every goal on the navigation action servers and wait until none is active.

        Goals other clients sent are cancelled too, because a stage that failed must leave the
        machine standing. Return whether the stop was confirmed.
        """
        if not self._rclpy.ok():
            return False
        try:
            ready = [c for c in self._cancel_all_clients if c.service_is_ready()]
            results = await asyncio.gather(
                *(
                    asyncio.wait_for(
                        _await_rclpy(c.call_async(self._CancelGoal.Request())),
                        timeout=self._send_goal_timeout_s,
                    )
                    for c in ready
                ),
                return_exceptions=True,
            )
            for client, outcome in zip(ready, results):
                if isinstance(outcome, BaseException):
                    logger.warning(
                        "[nav2_navigation] cancel-all on %s failed: %r", client.srv_name, outcome
                    )
            if await self._goals_end_within(_STOP_GRACE_S):
                return True
            async with self._zero_hold(True):
                if await self._goals_end_within(self._send_goal_timeout_s):
                    return True
            logger.warning("[nav2_navigation] stop_unconfirmed: a goal is still active")
            return False
        except Exception:  # noqa: BLE001 - the stage's own failure must still be reported
            logger.exception("[nav2_navigation] cancel-all failed")
            return False

    async def _goals_end_within(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self._any_goal_active():
            if time.monotonic() >= deadline:
                return False
            await asyncio.sleep(0.1)
        return True

    async def _build_path_poses(self, stage: mission_pb2.Stage) -> list[Any]:
        """Convert a coverage stage's planned route to map-frame poses.

        Placed from one ``/fromLL`` call and local offsets, exactly as a stage's waypoints are: the
        route holds thousands of points and a service round trip each would not finish.
        """
        anchor: tuple[float, float] | None = None
        origin: mission_pb2.WGS84Waypoint | None = None
        placed: list[tuple[float, float, float | None]] = []
        # Consecutive segments share an endpoint, so the joint is driven once.
        route = [
            waypoint
            for index, segment in enumerate(stage.coverage.segments)
            for waypoint in (segment.geometry[1:] if index else segment.geometry)
        ]
        for waypoint in route:
            if waypoint.WhichOneof("kind") != "wgs84":
                raise _FrameSwitchUnsupported(
                    "a planned coverage route carries only geographic waypoints"
                )
            if anchor is None:
                origin = waypoint.wgs84
                anchor = await self._wgs84_to_map_point(waypoint.wgs84)
            east, north = geo.enu_offset(
                origin.lat, origin.lon, waypoint.wgs84.lat, waypoint.wgs84.lon
            )
            heading = waypoint.wgs84.heading_deg if waypoint.wgs84.HasField("heading_deg") else None
            placed.append((anchor[0] + east, anchor[1] + north, heading))

        return [
            self._map_pose(x, y, heading)
            for x, y, heading in _filled(placed, self._cfg.max_pose_spacing_m)
        ]

    def _nearest_index(self, poses: list[Any]) -> int | None:
        """Return the index of the pose nearest the machine, or None when TF cannot say.

        None rather than a guess: a resume that cannot locate the machine has no idea how much of
        the field is already worked, and the plausible-looking guess is the head of the path, which
        would send a machine most of the way through a field back to its first swath.
        """
        position = self._robot_xy()
        if position is None:
            return None
        return min(range(len(poses)), key=lambda i: math.dist(_xy(poses[i]), position))

    def _within(self, pose: Any, tolerance_m: float) -> bool:
        """True when the machine stands within ``tolerance_m`` of ``pose``, False if TF cannot say."""
        position = self._robot_xy()
        return position is not None and math.dist(position, _xy(pose)) <= tolerance_m

    def _robot_xy(self) -> tuple[float, float] | None:
        """Return the machine's position in the map frame, or None when the transform is missing."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._cfg.frames.map, self._cfg.frames.base, self._rclpy.time.Time()
            )
        except Exception as exc:  # noqa: BLE001
            logger.warning("[nav2_navigation] map to base_footprint unavailable: %s", exc)
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    # ------------------------------------------------------------------
    # Pose construction
    # ------------------------------------------------------------------

    async def _build_poses(self, stage: mission_pb2.Stage) -> list[Any]:
        """Convert a stage's waypoints to map-frame poses.

        Every geographic waypoint in a stage is placed from a single ``/fromLL`` call: the first
        one is converted by the service, and the rest are offset from it on the local tangent
        plane. One call per stage rather than one per waypoint is what makes a covered field
        dispatchable at all, since a large one carries thousands and each call is a round trip.

        This assumes the map frame is metric and ENU-aligned, which is what ``navsat_transform``
        produces and what the site-local branch below already relies on.

        A coverage stage's swaths are flattened here into the order they are driven, and Nav2 plans
        between those points. The machine therefore covers the field without holding each swath to
        its line: reproducing the planned swath exactly means handing the path to a path-following
        controller rather than issuing a sequence of goals.
        """
        anchor: tuple[mission_pb2.WGS84Waypoint, float, float] | None = None
        poses: list[Any] = []
        for wp in proto_json.stage_waypoints(stage):
            kind = wp.WhichOneof("kind")
            if kind == "wgs84":
                if anchor is None:
                    x, y = await self._wgs84_to_map_point(wp.wgs84)
                    anchor = (wp.wgs84, x, y)
                origin, origin_x, origin_y = anchor
                east, north = geo.enu_offset(origin.lat, origin.lon, wp.wgs84.lat, wp.wgs84.lon)
                heading = wp.wgs84.heading_deg if wp.wgs84.HasField("heading_deg") else None
                poses.append(self._map_pose(origin_x + east, origin_y + north, heading))
            elif kind == "site_local":
                poses.append(self._site_local_to_pose_stamped(wp.site_local))
            else:
                raise _FrameSwitchUnsupported(f"unknown waypoint kind: {kind}")
        return poses

    async def _wgs84_to_map_point(self, wp: mission_pb2.WGS84Waypoint) -> tuple[float, float]:
        """Ask Nav2 where one geographic point lies in the map frame."""
        if not self._from_ll_client.wait_for_service(timeout_sec=self._from_ll_timeout_s):
            raise _FromLLFailed("/fromLL service not available")

        request = self._FromLL.Request()
        request.ll_point = self._GeoPoint(
            latitude=wp.lat,
            longitude=wp.lon,
            altitude=0.0,
        )
        try:
            response = await asyncio.wait_for(
                _await_rclpy(self._from_ll_client.call_async(request)),
                timeout=self._from_ll_timeout_s,
            )
        except asyncio.TimeoutError:
            raise _FromLLFailed(f"/fromLL call timed out after {self._from_ll_timeout_s}s")
        if response is None:
            raise _FromLLFailed("/fromLL returned no response")
        return response.map_point.x, response.map_point.y

    def _map_pose(self, x: float, y: float, heading_deg: float | None) -> Any:
        pose = self._PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = self._cfg.frames.map
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        yaw_east_rad = math.radians(90.0 - heading_deg) if heading_deg is not None else 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw_east_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_east_rad / 2.0)
        return pose

    def _site_local_to_pose_stamped(self, wp: mission_pb2.SiteLocalWaypoint) -> Any:
        pose = self._PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = self._cfg.frames.map
        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        pose.pose.position.z = 0.0
        theta = wp.theta if wp.HasField("theta") else 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        return pose


class _FrameSwitchUnsupported(Exception):
    """Raised when a stage demands a frame transition not yet implemented."""


class _FromLLFailed(Exception):
    """Raised when navsat_transform_node /fromLL service call fails."""
