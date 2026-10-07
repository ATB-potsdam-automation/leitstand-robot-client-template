"""One robot's presence on the Leitstand: everything declared on the session, in order."""

from __future__ import annotations

import logging
from typing import Any

from leitstand_client import keys, registration
from leitstand_client.config import RobotSpec
from leitstand_client.factsheet import FactsheetPublisher
from leitstand_client.mission_executor import MissionExecutor
from leitstand_client.navigation import Navigation
from leitstand_client.pose_source import PoseSource
from leitstand_client.recording import MissionRecording, PlatformPolicy, ScriptRecorder

logger = logging.getLogger(__name__)


class LeitstandClient:
    """Registers, serves the factsheet, runs missions and relays pose over one Zenoh session.

    Built with its adapters and chooses nothing: the caller decides what navigation and pose
    source the robot has. The caller also opened the session and closes it after ``stop()``.
    """

    def __init__(
        self,
        spec: RobotSpec,
        session: Any,
        navigation: Navigation,
        pose_source: PoseSource | None = None,
    ) -> None:
        self._spec = spec
        self._session = session
        self._navigation = navigation
        self._pose_source = pose_source
        self._metadata: Any = None
        self._factsheet: FactsheetPublisher | None = None
        self._executor: MissionExecutor | None = None
        self._liveliness: Any = None

    def start(self) -> None:
        """Declare everything, the liveliness token last: it is the readiness signal the backend watches."""
        robot_id = self._spec.id

        if self._spec.factsheet is not None:
            self._factsheet = FactsheetPublisher(self._session, robot_id, self._spec.factsheet)

        self._executor = MissionExecutor(
            self._session, robot_id, self._navigation, recording=_recording(self._spec)
        )
        self._executor.start()

        # After the executor, because the metadata reply names the run it is executing.
        metadata_key = keys.METADATA.format(robot_id=robot_id)
        self._metadata = self._session.declare_queryable(
            metadata_key,
            registration.metadata_handler(robot_id, metadata_key, self._executor.active_run_id),
        )
        logger.info("[client] metadata queryable declared: %s", metadata_key)

        if self._pose_source is not None:
            self._pose_source.start()

        self._liveliness = self._session.liveliness().declare_token(
            keys.ONLINE.format(robot_id=robot_id)
        )
        logger.info("[client] %s online", robot_id)

    def stop(self) -> None:
        """Undeclare in reverse, the liveliness token first so the backend sees offline before anything else goes quiet."""
        if self._liveliness is not None:
            try:
                self._liveliness.undeclare()
            except Exception as exc:  # noqa: BLE001
                logger.warning("[client] liveliness undeclare failed: %s", exc)
        if self._executor is not None:
            self._executor.close()
        try:
            self._navigation.close()
        except Exception as exc:  # noqa: BLE001
            logger.warning("[client] navigation close failed: %s", exc)
        if self._pose_source is not None:
            try:
                self._pose_source.close()
            except Exception as exc:  # noqa: BLE001
                logger.warning("[client] pose source close failed: %s", exc)
        if self._factsheet is not None:
            self._factsheet.close()
        if self._metadata is not None:
            try:
                self._metadata.undeclare()
            except Exception as exc:  # noqa: BLE001
                logger.warning("[client] metadata undeclare failed: %s", exc)
        logger.info("[client] %s offline", self._spec.id)


def _recording(spec: RobotSpec) -> MissionRecording | None:
    """The run recording this robot.yaml asks for, or None when it asks for none."""
    cfg = spec.recording
    if cfg is None:
        return None
    policy = (
        PlatformPolicy(cfg.platform_url, spec.id, timeout_s=cfg.policy_timeout_s)
        if cfg.platform_url
        else None
    )
    logger.info(
        "[client] missions are recorded (%s)",
        "streams from the data platform" if policy is not None else "streams from robot.yaml",
    )
    return MissionRecording(
        ScriptRecorder(cfg.start_cmd, cfg.stop_cmd, timeout_s=cfg.command_timeout_s),
        spec.id,
        policy=policy,
        streams=cfg.streams,
    )
