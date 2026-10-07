"""Mission executor: accepts dispatch, runs stages, reports state to the backend.

Zenoh surface:
  queryable  leitstand/robot/<id>/mission/_action/send_goal   (MissionDispatchRequest -> MissionDispatchResponse)
  queryable  leitstand/robot/<id>/mission/_action/cancel_goal (CancelRequest -> ControlResponse)
  queryable  leitstand/robot/<id>/mission/_action/pause       (ControlRequest -> ControlResponse)
  queryable  leitstand/robot/<id>/mission/_action/resume      (ControlRequest -> ControlResponse)
  publisher  leitstand/robot/<id>/mission/state               (MissionState)

A command is answered before it is acted on; the reply is a receipt, and the state frame
published once the command has taken effect is the confirmation the backend waits for.

Threading model:
  Zenoh callbacks arrive on Zenoh runtime threads (sync).
  Mission execution runs in a dedicated asyncio event loop thread.
  Callbacks bridge into the loop via asyncio.run_coroutine_threadsafe /
  call_soon_threadsafe. The navigation uses threading.Event for
  pause/resume so it can be set from any thread without loop involvement.
"""

from __future__ import annotations

import asyncio
import enum
import logging
import threading
import traceback
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from leitstand.robot.v1 import mission_pb2, mission_state_pb2, robot_control_pb2

from leitstand_client import keys, proto_json
from leitstand_client.navigation import Navigation, StageResult, is_immediate
from leitstand_client.recording import MissionRecording

logger = logging.getLogger(__name__)

_SEND_GOAL_KEY = keys.SEND_GOAL
_CANCEL_KEY = keys.CANCEL_GOAL
_PAUSE_KEY = keys.PAUSE
_RESUME_KEY = keys.RESUME
_STATE_KEY = keys.STATE

_HEARTBEAT_INTERVAL_S = 5.0

# Well under the backend's 10 s dispatch timeout, so an un-ready robot rejects instead of timing out.
_PREFLIGHT_TIMEOUT_S = 1.0

# How long close() lets a stage stop the machine before the task is cancelled outright: above
# Nav2's send-goal wait plus the zero-velocity hold, so a stop in progress is not cut short.
_CLOSE_TIMEOUT_S = 8.0

_TERMINAL_STAGE_STATUSES = frozenset(
    {
        mission_state_pb2.STAGE_STATUS_FINISHED,
        mission_state_pb2.STAGE_STATUS_FAILED,
        mission_state_pb2.STAGE_STATUS_CANCELLED,
        mission_state_pb2.STAGE_STATUS_SKIPPED,
    }
)


def _now() -> datetime:
    return datetime.now(timezone.utc)


def _query_payload_bytes(query: Any) -> bytes | None:
    payload = getattr(query, "payload", None)
    if payload is None:
        return None
    try:
        return bytes(payload.to_bytes())
    except AttributeError:
        try:
            return bytes(payload)
        except Exception:  # noqa: BLE001
            return None


class _DispatchDecision(enum.Enum):
    ACCEPT = "accept"  # idle robot: start the mission
    DUPLICATE = "duplicate"  # same mission already running: idempotent accept, no re-run
    BUSY = "busy"  # a different mission is running: reject


def _dispatch_decision(active_run_id: str | None, incoming_run_id: str) -> _DispatchDecision:
    """Decide a dispatch outcome from the run_id (idempotent on re-delivery).

    The id names one execution: a second run of the same mission carries a different id and
    is a new job, while a re-delivered dispatch of the run already executing carries the same
    one and is acknowledged without a second execution.
    """
    if active_run_id is None:
        return _DispatchDecision.ACCEPT
    if active_run_id == incoming_run_id:
        return _DispatchDecision.DUPLICATE
    return _DispatchDecision.BUSY


@dataclass
class _ActiveContext:
    """One run's state. The Zenoh thread writes cancel_mode and cancel_stage_index (single
    assignments); the loop thread writes everything else."""

    mission: mission_pb2.Mission
    task: asyncio.Task | None = field(default=None, init=False)
    # proto CancelMode int once requested; None means not cancelled (proto 0 is
    # UNSPECIFIED, not a safe "uncancelled" sentinel, so the field stays optional).
    cancel_mode: int | None = field(default=None, init=False)
    # The stage that was current when the cancel arrived; its cleanup runs even if it finished.
    cancel_stage_index: int | None = field(default=None, init=False)
    shutting_down: bool = field(default=False, init=False)
    stage_states: list[mission_state_pb2.StageState] = field(default_factory=list, init=False)
    stage_index: int = field(default=0, init=False)
    terminal_status: int | None = field(default=None, init=False)
    # Per stage, indexed like stage_states; _apply_progress publishes only on a 2 % change.
    last_published_progress: list[float] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        self.stage_states = [
            proto_json.make_stage_state(s.stage_id, mission_state_pb2.STAGE_STATUS_WAITING)
            for s in self.mission.stages
        ]
        self.last_published_progress = [0.0] * len(self.mission.stages)


class MissionExecutor:
    """Handles mission dispatch, execution, and state reporting for one robot."""

    def __init__(
        self,
        session: Any,
        robot_id: str,
        navigation: Navigation,
        *,
        state_publisher: Any = None,
        loop: asyncio.AbstractEventLoop | None = None,
        recording: MissionRecording | None = None,
    ) -> None:
        self._session = session
        self._robot_id = robot_id
        self._nav = navigation
        # None means this robot does not record with its missions.
        self._recording = recording

        self._lock = threading.Lock()
        self._active: _ActiveContext | None = None
        self._header_id = 0  # only modified from the asyncio loop thread

        # Asyncio loop in a dedicated daemon thread, unless a test hands one in.
        self._loop: asyncio.AbstractEventLoop | None = loop
        self._owns_loop = loop is None
        self._loop_ready = threading.Event()
        self._loop_thread = threading.Thread(
            target=self._run_loop, daemon=True, name="executor-loop"
        )

        # Zenoh resources (set in start(), the state publisher unless a test hands one in)
        self._state_pub: Any = state_publisher
        self._send_goal_qbl: Any = None
        self._cancel_qbl: Any = None
        self._pause_qbl: Any = None
        self._resume_qbl: Any = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._owns_loop:
            self._loop_thread.start()
            self._loop_ready.wait()

        if self._state_pub is None:
            self._state_pub = self._session.declare_publisher(
                _STATE_KEY.format(robot_id=self._robot_id)
            )
        self._send_goal_qbl = self._session.declare_queryable(
            _SEND_GOAL_KEY.format(robot_id=self._robot_id), self._handle_send_goal
        )
        self._cancel_qbl = self._session.declare_queryable(
            _CANCEL_KEY.format(robot_id=self._robot_id), self._handle_cancel_goal
        )
        self._pause_qbl = self._session.declare_queryable(
            _PAUSE_KEY.format(robot_id=self._robot_id), self._handle_pause
        )
        self._resume_qbl = self._session.declare_queryable(
            _RESUME_KEY.format(robot_id=self._robot_id), self._handle_resume
        )
        logger.info("[executor] started for robot %s", self._robot_id)

    def active_run_id(self) -> str | None:
        """The run being executed, or None; answered in the robot's metadata."""
        with self._lock:
            return self._active.mission.run_id if self._active is not None else None

    def close(self) -> None:
        # The task check runs on the loop thread, because only the loop's own ordering guarantees
        # that a dispatch accepted moments ago has started by then.
        with self._lock:
            ctx = self._active
            if ctx is not None:
                ctx.shutting_down = True
                ctx.cancel_mode = mission_pb2.CANCEL_MODE_IMMEDIATE

        if ctx is not None and self._loop is not None:
            fut = asyncio.run_coroutine_threadsafe(_stop_context(ctx), self._loop)
            try:
                fut.result(timeout=_CLOSE_TIMEOUT_S + 3.0)
            except Exception:  # noqa: BLE001
                pass

        # Undeclare Zenoh resources in reverse order.
        for res in (
            self._resume_qbl,
            self._pause_qbl,
            self._cancel_qbl,
            self._send_goal_qbl,
            self._state_pub,
        ):
            if res is not None:
                try:
                    res.undeclare()
                except Exception as exc:  # noqa: BLE001
                    logger.warning("[executor] undeclare failed: %s", exc)

        # Stop the asyncio loop, unless a test owns it.
        if self._owns_loop and self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._loop_thread.join(timeout=5.0)
        logger.info("[executor] closed")

    # ------------------------------------------------------------------
    # Asyncio loop thread
    # ------------------------------------------------------------------

    def _run_loop(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._loop_ready.set()
        try:
            loop.run_forever()
        finally:
            loop.close()

    # ------------------------------------------------------------------
    # Zenoh callbacks (called from Zenoh runtime threads)
    # ------------------------------------------------------------------

    def _handle_send_goal(self, query: Any) -> None:
        send_goal_key = _SEND_GOAL_KEY.format(robot_id=self._robot_id)
        try:
            payload = _query_payload_bytes(query)
            if payload is None:
                _reply_dispatch(query, send_goal_key, accepted=False, reason="empty payload")
                return
            try:
                request = proto_json.parse(payload, mission_pb2.MissionDispatchRequest)
            except Exception as exc:
                _reply_dispatch(
                    query, send_goal_key, accepted=False, reason=f"invalid payload: {exc}"
                )
                return

            mission = request.mission
            try:
                proto_json.validate_mission(mission)
            except Exception as exc:
                _reply_dispatch(
                    query, send_goal_key, accepted=False, reason=f"invalid mission: {exc}"
                )
                return

            with self._lock:
                active_id = self._active.mission.run_id if self._active is not None else None
                decision = _dispatch_decision(active_id, mission.run_id)
                if decision is _DispatchDecision.DUPLICATE:
                    # Re-delivered dispatch of the mission already running: idempotent
                    # accept with no second execution, keyed on the run_id.
                    _reply_dispatch(query, send_goal_key, accepted=True)
                    return
                if decision is _DispatchDecision.BUSY:
                    _reply_dispatch(
                        query,
                        send_goal_key,
                        accepted=False,
                        reason=f"busy with mission {active_id}",
                    )
                    return

            # Probe readiness before committing the slot. Run outside the lock: the
            # check blocks (bounded by _PREFLIGHT_TIMEOUT_S) only when the nav stack
            # is down, and holding the lock would stall a concurrent cancel/dispatch.
            reason = self._nav.check_ready(mission, _PREFLIGHT_TIMEOUT_S)
            if reason is not None:
                _reply_dispatch(query, send_goal_key, accepted=False, reason=reason)
                logger.info("[executor] rejected run %s: %s", mission.run_id, reason)
                return

            with self._lock:
                if self._active is not None:
                    # A dispatch raced in during the probe window and took the slot; the same
                    # run re-delivered is still the idempotent accept, not a rejection.
                    if self._active.mission.run_id == mission.run_id:
                        _reply_dispatch(query, send_goal_key, accepted=True)
                        return
                    _reply_dispatch(
                        query,
                        send_goal_key,
                        accepted=False,
                        reason=f"busy with run {self._active.mission.run_id}",
                    )
                    return
                # The pause flag lives on the navigation for the process's life, so a mission
                # accepted after a pause would otherwise start PAUSED and sit there. Cleared
                # before the run is visible, so a pause that lands now is not undone by it.
                self._nav.request_resume()
                ctx = _ActiveContext(mission=mission)
                self._active = ctx

            asyncio.run_coroutine_threadsafe(self._execute_mission(ctx), self._loop)
            _reply_dispatch(query, send_goal_key, accepted=True)
            logger.info("[executor] accepted run %s", mission.run_id)

        except Exception as exc:  # noqa: BLE001 - never crash the Zenoh thread
            logger.exception("[executor] send_goal error: %s", exc)
            try:
                _reply_dispatch(query, send_goal_key, accepted=False, reason="internal error")
            except Exception:  # noqa: BLE001
                pass

    def _handle_cancel_goal(self, query: Any) -> None:
        """Cancel the named mission, and only that one.

        The request names a mission because this queryable is addressed to the robot rather than
        to the work: a cancel aimed at a mission that has already ended would otherwise abort
        whatever the robot started next. An unparseable request cancels nothing, since a payload
        that cannot be read cannot be shown to name the mission that is running.
        """
        cancel_key = _CANCEL_KEY.format(robot_id=self._robot_id)
        try:
            payload = _query_payload_bytes(query)
            request = None
            if payload:
                try:
                    request = proto_json.parse(payload, mission_pb2.CancelRequest)
                except Exception:  # noqa: BLE001
                    request = None

            if request is None:
                logger.warning("[executor] cancel refused: unreadable request")
                _reply_control(
                    query, cancel_key, refusal=_REFUSAL_OTHER, reason="unreadable request"
                )
                return

            with self._lock:
                ctx = self._active
            if ctx is None or ctx.mission.run_id != request.run_id:
                logger.info("[executor] cancel refused: not executing run %s", request.run_id)
                _reply_control(query, cancel_key, refusal=_REFUSAL_NOT_EXECUTING)
                return

            _reply_control(query, cancel_key, applied=True)
            with self._lock:
                if self._active is ctx:
                    ctx.cancel_mode = request.mode
                    ctx.cancel_stage_index = ctx.stage_index
            logger.info(
                "[executor] cancel requested for %s (mode=%s)", request.run_id, request.mode
            )
            self._publish_soon(ctx)
        except Exception as exc:  # noqa: BLE001
            logger.exception("[executor] cancel_goal error: %s", exc)

    def _handle_pause(self, query: Any) -> None:
        self._handle_control(query, _PAUSE_KEY, "pause", self._nav.request_pause)

    def _handle_resume(self, query: Any) -> None:
        self._handle_control(query, _RESUME_KEY, "resume", self._nav.request_resume)

    def _handle_control(self, query: Any, key_template: str, name: str, act: Any) -> None:
        """Answer a pause or resume, then act; the frame after acting is the confirmation."""
        key = key_template.format(robot_id=self._robot_id)
        try:
            payload = _query_payload_bytes(query)
            try:
                request = proto_json.parse(payload or b"", robot_control_pb2.ControlRequest)
            except Exception:  # noqa: BLE001
                request = None
            if request is None or not request.run_id:
                _reply_control(query, key, refusal=_REFUSAL_OTHER, reason="unreadable request")
                return

            with self._lock:
                ctx = self._active
            if ctx is None or ctx.mission.run_id != request.run_id:
                logger.info("[executor] %s refused: not executing run %s", name, request.run_id)
                _reply_control(query, key, refusal=_REFUSAL_NOT_EXECUTING)
                return

            _reply_control(query, key, applied=True)
            act()
            logger.info("[executor] %s applied to run %s", name, request.run_id)
            self._publish_soon(ctx)
        except Exception as exc:  # noqa: BLE001
            logger.exception("[executor] %s error: %s", name, exc)

    def _publish_soon(self, ctx: _ActiveContext) -> None:
        """Publish a frame from the loop thread, unless the run has already ended."""
        if self._loop is None:
            return

        self._loop.call_soon_threadsafe(self._publish_unless_ended, ctx)

    # ------------------------------------------------------------------
    # Mission execution (runs in asyncio loop thread)
    # ------------------------------------------------------------------

    def _publish_unless_ended(self, ctx: _ActiveContext) -> None:
        if ctx.terminal_status is None:
            asyncio.ensure_future(self._publish_state(ctx))

    def _apply_progress(self, ctx: _ActiveContext, i: int, status: int, progress: float) -> None:
        """Update stage i's status and progress, then publish if the delta warrants it.

        Runs only on the asyncio loop thread, so no lock is needed against _publish_state and
        the heartbeat. Publishes only on a status change or a 2 % move, so header_id stays gap-free.
        """
        ss = ctx.stage_states[i]
        if ctx.terminal_status is not None or ss.status in _TERMINAL_STAGE_STATUSES:
            # A tick that arrives after the stage ended must not reopen it.
            return
        prev_status = ss.status
        # While the machine stands still for a pause, the stage is PAUSED whatever the feedback says.
        effective_status = (
            mission_state_pb2.STAGE_STATUS_PAUSED if self._nav.is_paused() else status
        )
        ss.status = effective_status
        ss.progress = min(max(progress, 0.0), 1.0)

        status_changed = effective_status != prev_status
        delta = abs(progress - ctx.last_published_progress[i])
        if status_changed or delta >= 0.02:
            ctx.last_published_progress[i] = progress
            self._publish_unless_ended(ctx)

    async def _execute_mission(self, ctx: _ActiveContext) -> None:
        ctx.task = asyncio.current_task()
        try:
            # Before the first stage, so the drive is recorded from its beginning. Started here
            # rather than in the dispatch reply, because the reply is a receipt the backend
            # waits for and the recorder's programs block.
            await self._record_run(start=True, run_id=ctx.mission.run_id)
            await self._publish_state(ctx)

            heartbeat = asyncio.create_task(self._heartbeat(ctx))
            cancelled_at: int | None = None
            paused_when_cancelled = False

            try:
                for i, stage in enumerate(ctx.mission.stages):
                    if ctx.cancel_mode is not None:
                        cancelled_at = i
                        break

                    ctx.stage_index = i
                    ss = ctx.stage_states[i]
                    ss.status = mission_state_pb2.STAGE_STATUS_INITIALIZING
                    ss.started_at.FromDatetime(_now())
                    await self._publish_state(ctx)

                    result: StageResult = await self._nav.execute_stage(
                        stage,
                        self._robot_id,
                        lambda: ctx.cancel_mode,
                        on_progress=lambda st, pr, _i=i, _ctx=ctx: self._loop.call_soon_threadsafe(
                            self._apply_progress, _ctx, _i, st, pr
                        ),
                    )

                    if result.status == mission_state_pb2.STAGE_STATUS_FINISHED and (
                        not is_immediate(ctx.cancel_mode)
                    ):
                        ss.status = mission_state_pb2.STAGE_STATUS_FINISHED
                        ss.ended_at.FromDatetime(_now())
                        ss.progress = 1.0
                        await self._publish_state(ctx)
                    elif ctx.cancel_mode is not None and result.error is None:
                        # Stage stopped for a cancel.
                        paused_when_cancelled = ss.status == mission_state_pb2.STAGE_STATUS_PAUSED
                        ss.status = mission_state_pb2.STAGE_STATUS_CANCELLED
                        ss.ended_at.FromDatetime(_now())
                        cancelled_at = i
                        break
                    else:
                        # A failure, also one during a cancel, ends the run with its error and no cleanup.
                        ss.status = mission_state_pb2.STAGE_STATUS_FAILED
                        ss.ended_at.FromDatetime(_now())
                        ctx.terminal_status = mission_state_pb2.MISSION_EXEC_STATUS_FAILED
                        error = _stage_error(result)
                        # Attach the stage_id so consumers can attribute the error
                        # to the exact stage without relying on current_stage_index.
                        error.references.append(
                            mission_state_pb2.ErrorReference(key="stage_id", value=stage.stage_id)
                        )
                        await self._publish_state(ctx, errors=[error])
                        return

                if ctx.shutting_down:
                    ctx.terminal_status = mission_state_pb2.MISSION_EXEC_STATUS_FAILED
                    await self._publish_state(ctx, errors=[_shutdown_error()])
                    return

                if ctx.cancel_mode is not None:
                    # The stage that was current when the cancel arrived gets its cleanup, even
                    # if it finished meanwhile; a stage never started gets none.
                    interrupted = (
                        ctx.cancel_stage_index
                        if ctx.cancel_stage_index is not None
                        else cancelled_at
                    )
                    # A machine paused for a person next to it must not drive off to clean up.
                    if (
                        interrupted is not None
                        and not paused_when_cancelled
                        and ctx.stage_states[interrupted].status
                        != mission_state_pb2.STAGE_STATUS_WAITING
                    ):
                        # Cleanup drives, so a pause left standing would hold it forever.
                        self._nav.request_resume()
                        await self._run_on_cancel(ctx, ctx.mission.stages[interrupted])
                    ctx.terminal_status = mission_state_pb2.MISSION_EXEC_STATUS_CANCELLED
                else:
                    ctx.terminal_status = mission_state_pb2.MISSION_EXEC_STATUS_SUCCEEDED
            finally:
                # Cancelled here rather than before the cleanup, so a long drive back keeps reporting.
                heartbeat.cancel()
                try:
                    await heartbeat
                except asyncio.CancelledError:
                    pass

            await self._publish_state(ctx)

        except asyncio.CancelledError:
            ctx.terminal_status = mission_state_pb2.MISSION_EXEC_STATUS_FAILED
            try:
                await self._publish_state(ctx, errors=[_shutdown_error()])
            except Exception:  # noqa: BLE001
                pass
            raise
        except Exception as exc:
            logger.exception("[executor] mission execution error: %s", exc)
            ctx.terminal_status = mission_state_pb2.MISSION_EXEC_STATUS_FAILED
            err = mission_state_pb2.Error(
                severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                type="executor_unhandled_exception",
                description=traceback.format_exc(),
            )
            try:
                await self._publish_state(ctx, errors=[err])
            except Exception:  # noqa: BLE001
                pass
        finally:
            # Every way out of the run passes here: finished, failed, cancelled, shutting down.
            await self._record_run(start=False, run_id=ctx.mission.run_id)
            with self._lock:
                if self._active is ctx:
                    self._active = None
            logger.info(
                "[executor] run %s ended: %s",
                ctx.mission.run_id,
                mission_state_pb2.MissionExecStatus.Name(ctx.terminal_status)
                if ctx.terminal_status is not None
                else "unset",
            )

    async def _record_run(self, *, start: bool, run_id: str) -> None:
        """Start or stop the run's recording in a thread; a recorder never ends a mission."""
        if self._recording is None:
            return
        rec = self._recording
        try:
            await asyncio.to_thread(rec.start if start else rec.stop, run_id)
        except Exception as exc:  # noqa: BLE001 - recording is never a mission's outcome
            logger.exception(
                "[executor] recording %s failed: %s", "start" if start else "stop", exc
            )

    async def _run_on_cancel(self, ctx: _ActiveContext, stage: mission_pb2.Stage) -> None:
        """Run on_cancel cleanup stages for `stage` (non-cancellable)."""
        if not stage.on_cancel:
            return
        for cleanup_stage in stage.on_cancel:
            ss = proto_json.make_stage_state(
                cleanup_stage.stage_id,
                mission_state_pb2.STAGE_STATUS_INITIALIZING,
                started_at=_now(),
            )
            ctx.stage_states.append(ss)
            await self._publish_state(ctx)

            result = await self._nav.execute_stage(
                cleanup_stage,
                self._robot_id,
                lambda: None,  # non-cancellable
            )

            ss.status = (
                mission_state_pb2.STAGE_STATUS_FINISHED
                if result.status == mission_state_pb2.STAGE_STATUS_FINISHED
                else mission_state_pb2.STAGE_STATUS_FAILED
            )
            ss.ended_at.FromDatetime(_now())
            await self._publish_state(ctx)

    async def _heartbeat(self, ctx: _ActiveContext) -> None:
        try:
            while True:
                await asyncio.sleep(_HEARTBEAT_INTERVAL_S)
                await self._publish_state(ctx)
        except asyncio.CancelledError:
            pass

    async def _publish_state(
        self,
        ctx: _ActiveContext,
        errors: list[mission_state_pb2.Error] | None = None,
    ) -> None:
        self._header_id += 1
        # Once the run is terminal, every frame (including the latched last one) must
        # report the terminal status, not a stale RUNNING; so the terminal value wins
        # over the live RUNNING/PAUSED computation.
        if ctx.terminal_status is not None:
            exec_status = ctx.terminal_status
            for ss in ctx.stage_states:
                if ss.status == mission_state_pb2.STAGE_STATUS_WAITING:
                    ss.status = mission_state_pb2.STAGE_STATUS_SKIPPED
        elif self._nav.is_paused():
            exec_status = mission_state_pb2.MISSION_EXEC_STATUS_PAUSED
        else:
            exec_status = mission_state_pb2.MISSION_EXEC_STATUS_RUNNING
        msg = mission_state_pb2.MissionState(
            run_id=ctx.mission.run_id,
            header_id=self._header_id,
            timestamp=proto_json.now_ts(),
            exec_status=exec_status,
            current_stage_index=ctx.stage_index,
            stage_states=list(ctx.stage_states),
            errors=errors or [],
        )
        try:
            self._state_pub.put(proto_json.to_json(msg))
        except Exception as exc:  # noqa: BLE001
            logger.warning("[executor] state publish failed: %s", exc)


async def _stop_context(ctx: _ActiveContext) -> None:
    """Let the stage stop the machine on its own cancel path; cancel the task only if it will not."""
    if ctx.task is None:
        return
    try:
        await asyncio.wait_for(asyncio.shield(ctx.task), timeout=_CLOSE_TIMEOUT_S)
        return
    except asyncio.TimeoutError:
        pass
    except (asyncio.CancelledError, Exception):  # noqa: BLE001
        return
    ctx.task.cancel()
    try:
        await ctx.task
    except (asyncio.CancelledError, Exception):  # noqa: BLE001
        pass


_REFUSAL_NOT_EXECUTING = robot_control_pb2.CONTROL_REFUSAL_NOT_EXECUTING_RUN
_REFUSAL_OTHER = robot_control_pb2.CONTROL_REFUSAL_OTHER


def _reply_control(
    query: Any,
    key: str,
    *,
    applied: bool = False,
    refusal: int | None = None,
    reason: str | None = None,
) -> None:
    response = robot_control_pb2.ControlResponse(applied=applied)
    if not applied:
        response.refusal = refusal if refusal is not None else _REFUSAL_OTHER
        response.reason = reason or (
            "not executing this run" if refusal == _REFUSAL_NOT_EXECUTING else "refused"
        )
    try:
        query.reply(key, proto_json.to_json(response))
    except Exception as exc:  # noqa: BLE001
        logger.warning("[executor] control reply failed: %s", exc)


def _stage_error(result: StageResult) -> mission_state_pb2.Error:
    """Return the error for a stage that ended without finishing and without a cancel.

    The navigation's own error wins. Without one, the error names what came back, so a failed
    run never reaches the operator without a reason.
    """
    if result.error is not None:
        return result.error
    if result.status == mission_state_pb2.STAGE_STATUS_FAILED:
        return mission_state_pb2.Error(
            severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
            type="navigation_failed",
            description="navigation reported the stage as failed without a reason",
        )
    if result.status == mission_state_pb2.STAGE_STATUS_CANCELLED:
        return mission_state_pb2.Error(
            severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
            type="navigation_cancelled",
            description="navigation stopped the stage although no cancel was requested",
        )
    statuses = mission_state_pb2.StageStatus
    name = (
        statuses.Name(result.status) if result.status in statuses.values() else str(result.status)
    )
    return mission_state_pb2.Error(
        severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
        type="unexpected_stage_result",
        description=f"navigation returned {name}, expected FINISHED or FAILED",
    )


def _shutdown_error() -> mission_state_pb2.Error:
    return mission_state_pb2.Error(
        severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
        type="client_shutdown",
        description="the client shut down while the run was executing",
    )


def _reply_dispatch(
    query: Any,
    key: str,
    *,
    accepted: bool,
    reason: str | None = None,
) -> None:
    response = mission_pb2.MissionDispatchResponse(accepted=accepted)
    if reason is not None:
        response.reason = reason
    try:
        query.reply(key, proto_json.to_json(response))
    except Exception as exc:  # noqa: BLE001
        logger.warning("[executor] dispatch reply failed: %s", exc)
