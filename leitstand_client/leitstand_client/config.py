"""Pydantic models for robot.yaml, the client's single source of configuration."""

from __future__ import annotations

from typing import Any, Literal
from uuid import UUID

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from leitstand_client.keys import validate_robot_id

WaypointKind = Literal["wgs84", "site_local"]


class SiteEntry(BaseModel):
    """A site this robot can drive in, with the Nav2 map its frame is anchored to.

    Declared in the factsheet, so the Leitstand offers the robot site-local missions; the map
    file is recorded for the frame switch, which the shipped Nav2 navigation does not perform yet.
    """

    model_config = ConfigDict(extra="forbid")

    site_id: UUID
    nav2_map_file: str


class NavigationCapabilitySpec(BaseModel):
    """Navigation capability: which waypoint frames this robot can execute."""

    model_config = ConfigDict(extra="forbid")

    supported_waypoint_kinds: list[WaypointKind] = Field(default_factory=lambda: ["wgs84"])


class PhysicalParametersSpec(BaseModel):
    """The machine's own measurements, declared so the backend never holds a second copy.

    No implement width here: an implement is mounted per job, so its width belongs to the mission.
    """

    model_config = ConfigDict(extra="forbid")

    track_width_m: float = Field(
        gt=0, description="Distance between the wheels in metres, centre to centre."
    )
    min_turning_radius_m: float = Field(
        ge=0, description="Zero for a robot that turns on the spot, such as a skid-steer."
    )


class CoverageCapabilitySpec(BaseModel):
    """Coverage capability: which waypoint frames this robot can drive swaths in.

    Declaring it is a claim about steering, not geometry: that the machine holds the swath line
    between its endpoints rather than taking any convenient path between them. Leave it out and
    the backend refuses to plan coverage for this robot.
    """

    model_config = ConfigDict(extra="forbid")

    supported_waypoint_kinds: list[WaypointKind] = Field(default_factory=lambda: ["wgs84"])


class FactsheetSpec(BaseModel):
    """Capability declaration published via the factsheet queryable.

    Per stage kind: presence of a block (e.g. ``navigation``) means the robot supports that
    kind. A stale or misspelt key fails at load rather than silently dropping a capability.
    """

    model_config = ConfigDict(extra="forbid")

    navigation: NavigationCapabilitySpec | None = Field(default_factory=NavigationCapabilitySpec)
    coverage: CoverageCapabilitySpec | None = None
    # Optional because a robot that only drives to stated waypoints needs none of it; a backend
    # that plans a path for this robot refuses rather than assuming a shape for it.
    physical_parameters: PhysicalParametersSpec | None = None
    sites: list[SiteEntry] = Field(default_factory=list)

    @model_validator(mode="after")
    def _site_local_needs_sites(self) -> FactsheetSpec:
        declared = [c for c in (self.navigation, self.coverage) if c is not None]
        if any("site_local" in c.supported_waypoint_kinds for c in declared) and not self.sites:
            raise ValueError("factsheet declares site_local but no sites (nav2 maps) configured")
        return self


class LeitstandSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    endpoint: str
    zenoh_mode: Literal["client", "peer"] = "client"


class RosSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    domain: int = 0
    node_name: str = "leitstand_client"
    # Simulation clocks only; a real robot leaves this false.
    use_sim_time: bool = False
    # Applied by the node before rclpy starts; empty leaves the environment's value in place.
    cyclonedds_uri: str = ""


class PoseSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    msg_type: Literal["gps_msgs/msg/GPSFix", "sensor_msgs/msg/NavSatFix"]
    topic: str
    # Septentrio receivers publish the heading in GPSFix.dip; any other producer leaves it at
    # 0.0, which would read as a heading of 90 degrees, so the field is only used when asked.
    heading_from_dip: bool = False

    @model_validator(mode="after")
    def _dip_is_a_gpsfix_field(self) -> PoseSpec:
        if self.heading_from_dip and self.msg_type != "gps_msgs/msg/GPSFix":
            raise ValueError("heading_from_dip applies to gps_msgs/msg/GPSFix only")
        return self


class TelemetrySpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    pose: PoseSpec | None = None


class FakeSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    speed_mps: float = Field(default=1.0, gt=0)


class RecordingSpec(BaseModel):
    """Recording that follows a run, started by the client with two programs of the robot's own.

    ``streams`` names the streams this robot has. Which of them a run records is held by the
    data platform (``platform_url``): a rule can switch one off, a stream without a rule is
    recorded. While the platform cannot be reached, all of them are recorded.
    """

    model_config = ConfigDict(extra="forbid")

    start_cmd: str = Field(min_length=1)
    stop_cmd: str = Field(min_length=1)
    streams: list[str] = Field(default_factory=list)
    # Empty means the client does not ask: it records `streams` and nothing else.
    platform_url: str = ""
    policy_timeout_s: float = Field(default=5.0, gt=0)
    command_timeout_s: float = Field(default=20.0, gt=0)


class RobotSpec(BaseModel):
    """Full robot.yaml schema."""

    model_config = ConfigDict(extra="forbid")

    id: str
    leitstand: LeitstandSpec
    ros: RosSpec = Field(default_factory=RosSpec)
    telemetry: TelemetrySpec | None = None
    factsheet: FactsheetSpec | None = None
    # nav2 and fake are what the ROS package ships; any other name is a Navigation the robot's
    # own package constructs, so the file can name it without a library change.
    navigation: str = Field(default="nav2", min_length=1)
    fake: FakeSpec = Field(default_factory=FakeSpec)
    # Parsed by the ROS package, which is the only thing that knows what Nav2 is.
    nav2: dict[str, Any] | None = None
    # Absent means this robot records nothing on its own; recordings are then started by hand
    # or by the data platform's own command.
    recording: RecordingSpec | None = None

    @field_validator("id")
    @classmethod
    def _id_is_valid(cls, value: str) -> str:
        return validate_robot_id(value)
