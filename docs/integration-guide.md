# Bringing a robot onto the Leitstand with this template

This is the walk from "nothing" to "our robot appears in the fleet view and drives a mission",
with the code you write for each kind of robot. The README has the reference material (every
config key, the Docker image, troubleshooting); this page is the order to do things in.

## What is in the box

| Directory | What it is | When you touch it |
|---|---|---|
| `leitstand_client/` | the library: registration, factsheet, mission execution, receipts, state reporting; the `Navigation` and `PoseSource` seams; `FakeNavigation` | never, unless the protocol changes |
| `leitstand_client_ros2/` | a ROS 2 node around the library: `Nav2Navigation` (Nav2 actions), `pose_relay.py` (a GNSS topic to the pose key), `client.launch.py`, `config/robot.yaml` | copy it when your robot has ROS 2 but not standard Nav2 |
| `examples/python_only/` | a 40-line `main.py` plus a `Navigation` and a `PoseSource` for a robot with no ROS | copy it when your robot has an SDK and no ROS |
| `docs/contract.md` | the wire protocol: ten Zenoh keys and their payloads | when you write a client in another language |
| `tools/mock_fleet.py` | several fake robots from one process | for testing the Leitstand itself |

The robot never decides mission state; it reports what it executes and the Leitstand keeps the
lifecycle. Everything the robot must say and answer is in `docs/contract.md`.

## Step 1: talk to the Leitstand without a machine (`navigation: fake`)

This proves the network, the router and the registration before Nav2 or an SDK is involved.
`FakeNavigation` walks a stage's waypoints in simulated time (default 1 m/s), reports progress,
pauses and cancels like a real machine, and moves nothing. With the robot's ROS sourced and
`python3-venv` installed (apt):

```
mkdir -p ws/src && cd ws
git clone <this repo> src/leitstand-robot-client-template
vcs import src < src/leitstand-robot-client-template/leitstand.repos
rosdep install --from-paths src --ignore-src -y
python3 -m venv --system-site-packages venv && touch venv/COLCON_IGNORE
venv/bin/pip install --upgrade pip   # 22.04's pip cannot build the contract's package metadata
venv/bin/pip install ./src/leitstand-robot-contract -r src/leitstand-robot-client-template/leitstand_client/requirements.txt
venv/bin/python -m colcon build --symlink-install
source install/setup.bash
```

(The README's installation section explains the virtual environment and `python -m colcon`.)

Copy `leitstand_client_ros2/config/robot.yaml`, set `id`, `leitstand.endpoint` (the router on
the Leitstand box, port 7447) and `navigation: fake`, then:

```
ros2 launch leitstand_client_ros2 client.launch.py config:=/path/to/robot.yaml
```

Within a few seconds the robot is online in the fleet view with its factsheet. Create a
navigation mission with two or three waypoints anywhere, dispatch it to the robot: the stage
timeline advances, Pause turns the run PAUSING then PAUSED within a second, Resume the reverse,
Cancel ends it. Measured against backend 0.7.0: a pause is confirmed in about 100 ms, resume
and cancel likewise.

Without ROS on the machine, the same check is `tools/mock_fleet.py --endpoint tcp/<router>:7447`
after installing the contract and the library (`pip install ../leitstand-robot-contract ./leitstand_client`).

## Step 2: a simulation, if you have one

Any Gazebo (or other) simulation of your robot that runs Nav2 with `navigate_through_poses`,
`navigate_to_pose`, `follow_path` and `robot_localization`'s `/fromLL` works exactly like the
real machine from the client's point of view. `leitstand_client_ros2/config/robot_sim.yaml` is
the profile for that case: `use_sim_time: true`, a `NavSatFix` pose on `/navsatfix`, a
factsheet that declares navigation and coverage, standard Nav2 action names. Start the
simulation, then the client with that profile:

```
ros2 launch leitstand_client_ros2 client.launch.py \
  config:=/path/to/leitstand_client_ros2/config/robot_sim.yaml
```

The robot appears online; a navigation mission drives the simulated robot through its
waypoints; a coverage mission (planned in the editor or by the assistant) is driven as one
`FollowPath` goal over the planned swaths. Registration, dispatch, pause, resume, both cancel
modes and the reconnect reconciliation are verified on this path against backend 0.7.0.

The same client runs in Docker (`README.md`, "Installation with Docker"): as a service,
`docker compose -f leitstand-robot-client-template/docker/docker-compose.yaml up -d --build`
from the workspace's `src/` with a `docker/.env` naming the ROS distribution, the DDS vendor
and the directory holding `robot.yaml`. Registration, dispatch and the receipts through the
container are verified. Driving Nav2 from the container is not yet.

## Step 3: your robot

Pick the row that matches the robot; each is a different amount of code.

### A. Standard Nav2 stack: no code

If the robot runs Nav2 with `navigate_through_poses` (and `follow_path` for coverage) and
`robot_localization` with `/fromLL`, the shipped node is the client. Write `robot.yaml`:

```yaml
id: my_robot_1                      # lowercase letters, digits, _ and -
leitstand:
  endpoint: tcp/192.168.126.10:7447 # the router on the Leitstand box
ros:
  domain: 0
telemetry:
  pose:
    msg_type: sensor_msgs/msg/NavSatFix
    topic: /gps/fix
factsheet:
  navigation:
    supported_waypoint_kinds: [wgs84]
  coverage:                         # only if the robot follows a swath as a line
    supported_waypoint_kinds: [wgs84]
  physical_parameters:
    track_width_m: 1.5
    min_turning_radius_m: 1.5       # the Leitstand refuses plans this robot cannot turn
navigation: nav2
# nav2: only if action names, frames or the controller id differ from the defaults
```

Launch as in step 1. The factsheet is what the Leitstand plans against: a robot without one is
online and never dispatched to; a robot without `coverage` is refused coverage missions; the
turning radius is checked against every plan before dispatch.

### B. ROS 2 with your own navigation: one class

Copy `leitstand_client_ros2/` to `leitstand_client_<robot>/`, keep `pose_relay.py`, `gnss.py`
and the launch file, and replace `nav2_navigation.py` with a class that implements the six methods:

```python
"""Navigation for <robot>: drives a stage through <your stack>."""

from __future__ import annotations

import asyncio
import threading
from typing import Callable

from leitstand.robot.v1 import mission_pb2, mission_state_pb2

from leitstand_client import StageResult, proto_json
from leitstand_client.navigation import CancelRequested, is_immediate


class RobotNavigation:
    def __init__(self, node) -> None:
        self._node = node
        self._stack = YourStack(node)       # whatever drives the machine: go_to, stop, is_up
        self._running = threading.Event()   # cleared by request_pause, set by request_resume
        self._running.set()
        self._stopped = threading.Event()   # set by the drive loop once the machine stands still

    async def execute_stage(
        self,
        stage: mission_pb2.Stage,
        robot_id: str,
        cancel_requested: CancelRequested,
        on_progress: Callable[[int, float], None] | None = None,
    ) -> StageResult:
        waypoints = proto_json.stage_waypoints(stage)      # WGS84 or site-local points, in order
        for index, waypoint in enumerate(waypoints, start=1):
            progress = (index - 1) / len(waypoints)
            if await self._hold_while_paused(cancel_requested, on_progress, progress):
                return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)
            mode = cancel_requested()
            if mode is not None:
                if is_immediate(mode):
                    await asyncio.to_thread(self._stack.stop)
                return StageResult(status=mission_state_pb2.STAGE_STATUS_FAILED)
            # Blocking SDK calls go into a thread: this loop also sends the heartbeat.
            ok = await asyncio.to_thread(self._stack.go_to, waypoint.wgs84.lat, waypoint.wgs84.lon)
            if not ok:
                return StageResult(
                    status=mission_state_pb2.STAGE_STATUS_FAILED,
                    error=mission_state_pb2.Error(
                        severity=mission_state_pb2.ERROR_SEVERITY_FATAL,
                        type="goal_rejected",
                        description=f"waypoint {index} was refused by the stack",
                    ),
                )
            if on_progress is not None:
                on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, index / len(waypoints))
        return StageResult(status=mission_state_pb2.STAGE_STATUS_FINISHED)

    async def _hold_while_paused(self, cancel_requested, on_progress, progress) -> bool:
        if self._running.is_set():
            return False
        await asyncio.to_thread(self._stack.stop)          # the machine stands still now
        self._stopped.set()
        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_PAUSED, progress)
        while not self._running.is_set():
            if cancel_requested() is not None:
                self._stopped.clear()
                return True
            await asyncio.sleep(0.1)
        self._stopped.clear()
        if on_progress is not None:
            on_progress(mission_state_pb2.STAGE_STATUS_RUNNING, progress)
        return False

    def check_ready(self, mission: mission_pb2.Mission, timeout_s: float) -> str | None:
        return None if self._stack.is_up() else "navigation stack is not running"

    def request_pause(self) -> None:      # Zenoh thread: ask, do not wait
        self._running.clear()

    def request_resume(self) -> None:
        self._running.set()

    def is_paused(self) -> bool:          # true only while the machine stands still
        return self._stopped.is_set()

    def close(self) -> None:
        self._stack.stop()
```

Then construct it in `node.py`:

```python
from leitstand_client_<robot>.robot_navigation import RobotNavigation


def _navigation(spec: RobotSpec, nav2_config, node):
    if spec.navigation == "<robot>":
        return RobotNavigation(node)
    ...
```

and set `navigation: <robot>` in `robot.yaml`. The threading rules the class has to keep:
`execute_stage` and `is_paused` run on the client's asyncio loop, which also sends the 5 s
heartbeat and every state frame, so anything that blocks there silences the robot;
`check_ready`, `request_pause`, `request_resume` run on the Zenoh thread and must return at
once; `close` runs on the main thread at shutdown. An exception out of `execute_stage` ends the
run as FAILED with the traceback as the error text, which is acceptable but not what an operator
wants to read: return a `StageResult` with an `Error` instead.

`Nav2Navigation` in the shipped package is the full reference (readiness probe, `/fromLL`
projection, a pause that stops the machine, resume that re-enters a coverage path behind the
machine, structured errors).

### C. No ROS, a Python SDK: three files

`examples/python_only/` is the complete example. `main.py` loads the config, opens the Zenoh
session and hands two objects to the client:

```python
spec = load_spec("robot.yaml")
session = open_session(build_zenoh_config(spec.leitstand))
client = LeitstandClient(spec, session, MyNavigation(), MyPoseSource(session, spec.id, lat, lon))
client.start()
...           # wait for SIGINT
client.stop()
session.close()
```

`MyNavigation` is the class from B without ROS (the example's `drive_leg` is where the SDK call
goes); `MyPoseSource` publishes a `Pose` on the robot's key once a second:

```python
pose = telemetry_pb2.Pose(lat=lat, lon=lon)
pose.timestamp.FromNanoseconds(time.time_ns())
session.put(keys.POSE.format(robot_id=robot_id), proto_json.to_json(pose),
            encoding=zenoh.Encoding.APPLICATION_JSON)
```

The config is the same `robot.yaml` without the `ros`, `telemetry` and `nav2` blocks:

```yaml
id: python_example
leitstand:
  endpoint: tcp/localhost:7447
factsheet:
  navigation:
    supported_waypoint_kinds: [wgs84]
```

### D. Another language: the protocol

`docs/contract.md` is the whole protocol: ten Zenoh keys under `leitstand/robot/<id>/`, proto
messages from `leitstand-robot-contract` as JSON, the order to declare them in, the timeouts the
backend waits (10 s dispatch, 5 s cancel, 3 s pause and resume), and what a receipt means. The
Python library is then a worked reference, nothing more.

## Optional: recording the run (data platform)

A robot that delivers its measurements to the data platform can have the client start and stop
the recording with the run, so a recording belongs to a mission without anybody asking for it
and its `run_id` is exact. Add one block to `robot.yaml`:

```yaml
recording:
  start_cmd: /opt/record/start.sh      # called as: start_cmd <name> <stream>[,<stream>...]
  stop_cmd: /opt/record/stop.sh        # called as: stop_cmd <name>
  platform_url: http://platform:8000   # the rules per stream are read from there
  streams: [telemetry, camera]         # the streams this robot has
```

Leave the block out and the client records nothing; recordings are then started by hand or by
the data platform's own command. The two programs are the robot's own (a systemd unit, a launch
file, a node of yours); only the calls and their arguments are fixed, and example scripts for
`ros2 bag record` are in the data platform's `docs/recording-control.md`. The recording name is
`<robot_id>-<run_id>`, and the run id is also in the environment as `LEITSTAND_RUN_ID`.

`streams` names what this robot has, which changes when the machine changes. Which of them a
run records is held by the data platform (`GET /v1/robots/<id>/policy`, field `record`), so
that setting lives in one place and can be changed without touching the robot: a rule switches
a stream off, a stream the platform holds no rule for is recorded, and while the platform
cannot be reached all of them are recorded. A recorder that fails is logged and the mission
goes on: recording never decides a run's outcome.

## What the Leitstand expects from any client

- **Identity**: an `id` of lowercase letters, digits, `_` and `-`; a liveliness token on
  `online`; `metadata` answering `{"id": ..., "active_run_id": ...}` so a reconnect tells the
  backend which run the robot still holds.
- **Factsheet**: what the robot can do. Navigation and coverage are separate capabilities; the
  physical parameters are what plans are checked against.
- **Missions**: a dispatch names a `run_id`; the same run sent twice is accepted once; a cancel,
  pause or resume names the run and is answered with a receipt before the machine acts, and a
  state frame follows as soon as it has.
- **State**: a frame per change and a heartbeat every 5 s while a run executes, each stage with
  its status and progress; after a cancel the cancelled stage is `CANCELLED`, the rest `SKIPPED`.
- **Pose**: optional, one `Pose` per second is plenty.

## Before the demo

1. `make contract-check` says `leitstand-robot-contract 0.4.0 ok`.
2. `pytest` is green, and `ruff check .`.
3. Step 1 with `navigation: fake` against the demo Leitstand: online, dispatch, pause, cancel.
4. Step 2 in the simulation, one navigation mission and one coverage mission.
