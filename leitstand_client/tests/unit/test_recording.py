"""A run records what the platform's rules ask for, and a recorder never ends a mission."""

from __future__ import annotations

import asyncio
import json
from typing import Any

from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client.config import RobotSpec
from leitstand_client.mission_executor import MissionExecutor, _ActiveContext
from leitstand_client.navigation import FakeNavigation
from leitstand_client.recording import (
    MissionRecording,
    PlatformPolicy,
    RecordingError,
    ScriptRecorder,
)

RUN = "11111111-1111-4111-8111-111111111111"


class _Publisher:
    def put(self, payload: bytes) -> None:  # noqa: D102 - frames are not what this file pins
        pass


class _FakeRecorder:
    """Records the calls instead of a bag."""

    def __init__(self, *, fail_on_start: bool = False) -> None:
        self.started: list[tuple[str, list[str], str]] = []
        self.stopped: list[str] = []
        self._fail_on_start = fail_on_start

    def start(self, name: str, streams: list[str], run_id: str) -> None:
        if self._fail_on_start:
            raise RecordingError("no recorder on this robot")
        self.started.append((name, list(streams), run_id))

    def stop(self, name: str) -> None:
        self.stopped.append(name)


class _FakePolicy:
    def __init__(self, answer: dict[str, bool] | None) -> None:
        self._answer = answer

    def rules(self) -> dict[str, bool] | None:
        return self._answer


def _stage(stage_id: str) -> mission_pb2.Stage:
    stage = mission_pb2.Stage(stage_id=stage_id, kind=mission_pb2.STAGE_KIND_NAVIGATION)
    for i in range(2):
        wp = stage.navigation.waypoints.add()
        wp.wgs84.lat = 52.0 + 0.001 * i
        wp.wgs84.lon = 8.0
    return stage


def _run_mission(recording: MissionRecording | None, *, cancel: bool = False) -> int | None:
    async def run() -> int | None:
        executor = MissionExecutor(
            session=object(),
            robot_id="r1",
            navigation=FakeNavigation(speed_mps=1000.0, min_leg_s=0.01, tick_s=0.005),
            state_publisher=_Publisher(),
            loop=asyncio.get_running_loop(),
            recording=recording,
        )
        ctx = _ActiveContext(mission=mission_pb2.Mission(run_id=RUN, stages=[_stage("a")]))
        if cancel:
            ctx.cancel_mode = mission_pb2.CANCEL_MODE_IMMEDIATE
            ctx.cancel_stage_index = 0
        await executor._execute_mission(ctx)
        return ctx.terminal_status

    return asyncio.run(run())


def test_a_run_is_recorded_from_its_start_until_it_ends() -> None:
    recorder = _FakeRecorder()
    recording = MissionRecording(
        recorder, "r1", policy=_FakePolicy({"telemetry": True}), streams=["telemetry"]
    )

    status = _run_mission(recording)

    assert status == mission_state_pb2.MISSION_EXEC_STATUS_SUCCEEDED
    assert recorder.started == [(f"r1-{RUN}", ["telemetry"], RUN)]
    assert recorder.stopped == [f"r1-{RUN}"]


def test_a_cancelled_run_stops_its_recording_too() -> None:
    recorder = _FakeRecorder()
    recording = MissionRecording(
        recorder, "r1", policy=_FakePolicy({"telemetry": True}), streams=["telemetry"]
    )

    status = _run_mission(recording, cancel=True)

    assert status == mission_state_pb2.MISSION_EXEC_STATUS_CANCELLED
    assert recorder.stopped == [f"r1-{RUN}"]


def test_the_platform_decides_what_is_recorded() -> None:
    # "soil moisture yes, camera no": the camera's rule switches the stream off.
    recorder = _FakeRecorder()
    recording = MissionRecording(
        recorder,
        "r1",
        policy=_FakePolicy({"telemetry": True, "camera": False}),
        streams=["telemetry", "camera"],
    )

    _run_mission(recording)

    assert recorder.started[0][1] == ["telemetry"]


def test_a_robot_without_rules_records_everything_it_has() -> None:
    # The platform treats a stream without a rule as recorded; so does this, or a robot whose
    # rules were never set would record nothing while the platform thinks it records all.
    recorder = _FakeRecorder()
    recording = MissionRecording(
        recorder, "r1", policy=_FakePolicy({}), streams=["telemetry", "camera"]
    )

    _run_mission(recording)

    assert recorder.started[0][1] == ["camera", "telemetry"]


def test_a_rule_can_ask_for_a_stream_the_file_does_not_name() -> None:
    recorder = _FakeRecorder()
    recording = MissionRecording(
        recorder, "r1", policy=_FakePolicy({"camera": True}), streams=["telemetry"]
    )

    _run_mission(recording)

    assert recorder.started[0][1] == ["camera", "telemetry"]


def test_without_an_answer_the_robots_own_streams_are_recorded() -> None:
    # Missing data cannot be recovered afterwards; an unwanted recording can be dropped.
    recorder = _FakeRecorder()
    recording = MissionRecording(
        recorder, "r1", policy=_FakePolicy(None), streams=["telemetry", "camera"]
    )

    _run_mission(recording)

    assert recorder.started[0][1] == ["telemetry", "camera"]


def test_a_recorder_that_fails_does_not_fail_the_mission() -> None:
    recorder = _FakeRecorder(fail_on_start=True)
    recording = MissionRecording(
        recorder, "r1", policy=_FakePolicy({"telemetry": True}), streams=["telemetry"]
    )

    status = _run_mission(recording)

    assert status == mission_state_pb2.MISSION_EXEC_STATUS_SUCCEEDED
    # Nothing was started, so nothing is stopped and no stale name is addressed.
    assert recorder.started == []
    assert recorder.stopped == []


def test_a_robot_without_the_recording_block_records_nothing() -> None:
    assert _run_mission(None) == mission_state_pb2.MISSION_EXEC_STATUS_SUCCEEDED

    spec = RobotSpec.model_validate({"id": "r1", "leitstand": {"endpoint": "tcp/localhost:7447"}})
    assert spec.recording is None


def test_nothing_to_record_starts_no_recorder() -> None:
    # Every stream the robot has is switched off by a rule.
    recorder = _FakeRecorder()
    recording = MissionRecording(
        recorder, "r1", policy=_FakePolicy({"telemetry": False}), streams=["telemetry"]
    )

    _run_mission(recording)

    assert recorder.started == []


def test_the_policy_reads_the_record_flag_per_stream(monkeypatch: Any) -> None:
    body = json.dumps(
        {
            "robot_id": "r1",
            "streams": {
                "telemetry": {"record": True, "transfer": "now"},
                "camera": {"record": False, "transfer": "never"},
            },
        }
    ).encode()

    class _Answer:
        def read(self) -> bytes:
            return body

        def __enter__(self) -> "_Answer":
            return self

        def __exit__(self, *exc: object) -> None:
            return None

    monkeypatch.setattr(
        "leitstand_client.recording.urllib.request.urlopen", lambda url, timeout: _Answer()
    )
    assert PlatformPolicy("http://platform:8000", "r1").rules() == {
        "telemetry": True,
        "camera": False,
    }


def test_an_unreachable_platform_answers_none(monkeypatch: Any) -> None:
    def _boom(url: str, timeout: float) -> None:
        raise OSError("connection refused")

    monkeypatch.setattr("leitstand_client.recording.urllib.request.urlopen", _boom)
    assert PlatformPolicy("http://platform:8000", "r1").rules() is None


def test_a_missing_program_is_reported_as_a_recording_error() -> None:
    recorder = ScriptRecorder("/nonexistent/start.sh", "/nonexistent/stop.sh", timeout_s=2.0)
    try:
        recorder.start("r1-run", ["telemetry"], RUN)
    except RecordingError as exc:
        assert "not found" in str(exc)
    else:  # pragma: no cover - the program really does not exist
        raise AssertionError("a missing program must be reported")
