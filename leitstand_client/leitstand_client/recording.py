"""Recording that follows a run: the client starts the recorder when it accepts a mission.

The robot is the only place with ROS, and the client is the only place that knows the run, so
a recording that belongs to a mission is started and stopped here. The recorder itself is not
ROS code: it is the same pair of programs the data platform's edge agent calls, so the
behaviour is identical whoever starts it.

Which streams a run records is held by the data platform (``GET /v1/robots/<id>/policy``), so
the setting lives in one place and can be changed without touching the robot. ``robot.yaml``
names the streams this robot has; a rule switches one off. A stream the platform holds no rule
for is recorded, which is what the platform itself does with an unruled stream, so a robot
whose rules were never set records everything it has rather than nothing. When the platform
cannot be reached at all, the configured streams are recorded: missing data cannot be recovered
afterwards, an unwanted recording can be dropped.

Recording never decides a mission's outcome. Every failure here is logged and the run goes on.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import urllib.error
import urllib.request
from typing import Protocol

logger = logging.getLogger(__name__)


class RecordingError(Exception):
    """A recorder program failed, or none is configured."""


class Recorder(Protocol):
    """Starts and stops a recording on this robot. Blocking; called off the event loop."""

    def start(self, name: str, streams: list[str], run_id: str) -> None: ...

    def stop(self, name: str) -> None: ...


class ScriptRecorder:
    """Calls two programs: ``start_cmd <name> <stream>[,<stream>...]`` and ``stop_cmd <name>``.

    How recording is started differs per robot (systemd unit, launch file, a node of your
    own), so only the two calls and their arguments are fixed. The run id is passed in the
    environment as ``LEITSTAND_RUN_ID`` for scripts that want it in the recording's metadata.
    """

    def __init__(self, start_cmd: str, stop_cmd: str, *, timeout_s: float = 20.0) -> None:
        self._start = start_cmd
        self._stop = stop_cmd
        self._timeout_s = timeout_s

    def start(self, name: str, streams: list[str], run_id: str) -> None:
        self._run([self._start, name, ",".join(streams)], {"LEITSTAND_RUN_ID": run_id})

    def stop(self, name: str) -> None:
        self._run([self._stop, name], {})

    def _run(self, argv: list[str], env: dict[str, str]) -> None:
        try:
            done = subprocess.run(  # noqa: S603 - the program is configured, not user input
                argv,
                env={**os.environ, **env},
                capture_output=True,
                text=True,
                timeout=self._timeout_s,
                check=False,
            )
        except FileNotFoundError as exc:
            raise RecordingError(f"{argv[0]} not found") from exc
        except subprocess.TimeoutExpired as exc:
            raise RecordingError(f"{argv[0]} did not return within {self._timeout_s} s") from exc
        if done.returncode != 0:
            detail = (done.stderr or done.stdout or "").strip()[:500]
            raise RecordingError(f"{argv[0]} exited with {done.returncode}: {detail}")


class PlatformPolicy:
    """Reads which streams to record from the data platform. Blocking; no new dependency."""

    def __init__(self, base_url: str, robot_id: str, *, timeout_s: float = 5.0) -> None:
        self._url = f"{base_url.rstrip('/')}/v1/robots/{robot_id}/policy"
        self._timeout_s = timeout_s

    def rules(self) -> dict[str, bool] | None:
        """Record yes/no per stream the platform holds a rule for, or None when it did not answer.

        An empty answer is an answer: the robot has no rules, not "record nothing".
        """
        try:
            with urllib.request.urlopen(self._url, timeout=self._timeout_s) as answer:  # noqa: S310
                body = json.loads(answer.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError, TimeoutError) as exc:
            logger.warning("[recording] policy from %s unavailable: %s", self._url, exc)
            return None
        streams = body.get("streams")
        if not isinstance(streams, dict):
            logger.warning("[recording] policy from %s has no streams", self._url)
            return None
        return {
            name: rule.get("record", True) is True
            for name, rule in streams.items()
            if isinstance(rule, dict)
        }


class MissionRecording:
    """Decides what a run records and keeps the name it was started under.

    The name is kept rather than recomputed, so a stop always addresses what was started even
    if the template or the clock would give a different answer.
    """

    def __init__(
        self,
        recorder: Recorder,
        robot_id: str,
        *,
        policy: PlatformPolicy | None = None,
        streams: list[str] | None = None,
    ) -> None:
        self._recorder = recorder
        self._robot_id = robot_id
        self._policy = policy
        self._streams = list(streams or [])
        self._open: dict[str, str] = {}  # run_id -> recording name

    def name_for(self, run_id: str) -> str:
        return f"{self._robot_id}-{run_id}"

    def streams_for_run(self) -> list[str]:
        """The robot's streams, minus the ones a rule switches off, plus the ones a rule asks for."""
        rules = self._policy.rules() if self._policy is not None else None
        if rules is None:
            return list(self._streams)
        asked = {name for name, record in rules.items() if record}
        return [s for s in sorted(set(self._streams) | asked) if rules.get(s, True)]

    def start(self, run_id: str) -> None:
        """Start recording this run. Logged and swallowed on failure; the run goes on."""
        if run_id in self._open:
            return
        streams = self.streams_for_run()
        if not streams:
            logger.info("[recording] run %s records nothing (no stream asked for)", run_id)
            return
        name = self.name_for(run_id)
        try:
            self._recorder.start(name, streams, run_id)
        except RecordingError as exc:
            logger.error("[recording] run %s not recorded: %s", run_id, exc)
            return
        self._open[run_id] = name
        logger.info("[recording] run %s recording %s as %s", run_id, ",".join(streams), name)

    def stop(self, run_id: str) -> None:
        """Stop this run's recording, if one was started."""
        name = self._open.pop(run_id, None)
        if name is None:
            return
        try:
            self._recorder.stop(name)
        except RecordingError as exc:
            logger.error("[recording] stopping %s failed: %s", name, exc)
            return
        logger.info("[recording] stopped %s", name)
