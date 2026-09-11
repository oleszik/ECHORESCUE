"""ROS-independent geometry and Gazebo evaluation contracts for v0.14.5.

This module scores simulator evidence.  It is deliberately not imported by the
waypoint controller or either MAVLink command node.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import isfinite, sqrt
from pathlib import Path
from typing import Any, Mapping, Sequence


Vector3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class AxisAlignedBox:
    name: str
    classification: str
    center: Vector3
    size: Vector3
    contact_topic: str

    def __post_init__(self) -> None:
        if not self.name or not self.contact_topic.startswith("/"):
            raise ValueError("indoor entities require a name and absolute contact topic")
        if self.classification not in {"floor", "ceiling", "wall", "obstacle"}:
            raise ValueError(f"unsupported indoor entity classification: {self.classification}")
        if not all(isfinite(value) for value in (*self.center, *self.size)):
            raise ValueError("indoor entity geometry must be finite")
        if any(value <= 0.0 for value in self.size):
            raise ValueError("indoor entity dimensions must be positive")

    @property
    def minimum(self) -> Vector3:
        return tuple(center - size / 2.0 for center, size in zip(self.center, self.size))  # type: ignore[return-value]

    @property
    def maximum(self) -> Vector3:
        return tuple(center + size / 2.0 for center, size in zip(self.center, self.size))  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class Doorway:
    name: str
    plane_coordinate_m: float
    center_y_m: float
    width_m: float
    bottom_m: float
    height_m: float
    crossing_hysteresis_m: float

    @property
    def minimum_y_m(self) -> float:
        return self.center_y_m - self.width_m / 2.0

    @property
    def maximum_y_m(self) -> float:
        return self.center_y_m + self.width_m / 2.0

    @property
    def top_m(self) -> float:
        return self.bottom_m + self.height_m


@dataclass(frozen=True, slots=True)
class ReferenceTarget:
    target_id: str
    east_offset_m: float
    north_offset_m: float
    altitude_above_launch_m: float

    def absolute(self, launch: Vector3) -> Vector3:
        return (
            launch[0] + self.east_offset_m,
            launch[1] + self.north_offset_m,
            launch[2] + self.altitude_above_launch_m,
        )


@dataclass(frozen=True, slots=True)
class IndoorReferenceConfig:
    schema_version: str
    milestone: str
    world_version: str
    world_name: str
    world_sdf: Path
    model_name: str
    launch_world_enu: Vector3
    allowed_minimum: Vector3
    allowed_maximum: Vector3
    doorway: Doorway
    entities: tuple[AxisAlignedBox, ...]
    vehicle_radius_m: float
    safety_margin_m: float
    required_center_clearance_m: float
    launch_ground_contact_radius_m: float
    ground_contact_maximum_center_z_m: float
    takeoff_altitude_m: float
    targets: tuple[ReferenceTarget, ...]
    pose_topic: str
    sample_interval_s: float
    far_side_minimum_x_m: float
    return_region_radius_m: float
    agreement_tolerance_m: float

    @property
    def obstacle(self) -> AxisAlignedBox:
        return next(entity for entity in self.entities if entity.classification == "obstacle")

    @property
    def contact_topics(self) -> tuple[str, ...]:
        return tuple(entity.contact_topic for entity in self.entities)


def _vector(values: Sequence[Any], name: str, length: int = 3) -> Vector3:
    if len(values) != length:
        raise ValueError(f"{name} must contain {length} values")
    converted = tuple(float(value) for value in values)
    if not all(isfinite(value) for value in converted):
        raise ValueError(f"{name} must be finite")
    return converted  # type: ignore[return-value]


def load_indoor_config(path: Path | str) -> tuple[dict[str, Any], IndoorReferenceConfig]:
    path = Path(path)
    raw: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    world = raw["world"]
    geometry = raw["geometry"]
    safety = raw["safety"]
    mission = raw["mission"]
    evaluation = raw["evaluation"]
    doorway_raw = geometry["doorway"]
    if doorway_raw.get("plane_axis") != "x":
        raise ValueError("v0.14.5 supports one x-normal doorway plane")
    config = IndoorReferenceConfig(
        schema_version=str(raw["schema_version"]),
        milestone=str(raw["milestone"]),
        world_version=str(world["version"]),
        world_name=str(world["name"]),
        world_sdf=Path(str(world["sdf_path"])),
        model_name=str(world["model_name"]),
        launch_world_enu=_vector(world["launch_pose_world_enu"][:3], "launch pose"),
        allowed_minimum=_vector(geometry["allowed_flight_volume"]["minimum"], "allowed minimum"),
        allowed_maximum=_vector(geometry["allowed_flight_volume"]["maximum"], "allowed maximum"),
        doorway=Doorway(
            name=str(doorway_raw["name"]),
            plane_coordinate_m=float(doorway_raw["plane_coordinate_m"]),
            center_y_m=float(doorway_raw["center_y_m"]),
            width_m=float(doorway_raw["width_m"]),
            bottom_m=float(doorway_raw["bottom_m"]),
            height_m=float(doorway_raw["height_m"]),
            crossing_hysteresis_m=float(doorway_raw["crossing_hysteresis_m"]),
        ),
        entities=tuple(
            AxisAlignedBox(
                name=str(item["name"]), classification=str(item["classification"]),
                center=_vector(item["center"], f"{item['name']} center"),
                size=_vector(item["size"], f"{item['name']} size"),
                contact_topic=str(item["contact_topic"]),
            )
            for item in geometry["entities"]
        ),
        vehicle_radius_m=float(safety["conservative_vehicle_radius_m"]),
        safety_margin_m=float(safety["safety_margin_m"]),
        required_center_clearance_m=float(safety["required_nominal_center_clearance_m"]),
        launch_ground_contact_radius_m=float(safety["launch_ground_contact_radius_m"]),
        ground_contact_maximum_center_z_m=float(safety["ground_contact_maximum_center_z_m"]),
        takeoff_altitude_m=float(mission["takeoff_altitude_m"]),
        targets=tuple(
            ReferenceTarget(
                str(item["target_id"]), float(item["east_offset_m"]),
                float(item["north_offset_m"]), float(item["altitude_above_launch_m"]),
            )
            for item in mission["targets"]
        ),
        pose_topic=str(evaluation["pose_topic"]),
        sample_interval_s=float(evaluation["sample_interval_s"]),
        far_side_minimum_x_m=float(evaluation["far_side_minimum_x_m"]),
        return_region_radius_m=float(evaluation["return_region_radius_m"]),
        agreement_tolerance_m=float(evaluation["telemetry_world_agreement_tolerance_m"]),
    )
    validate_indoor_config(config)
    return raw, config


def point_box_distance(point: Vector3, box: AxisAlignedBox) -> float:
    squared = 0.0
    for value, minimum, maximum in zip(point, box.minimum, box.maximum):
        delta = minimum - value if value < minimum else value - maximum if value > maximum else 0.0
        squared += delta * delta
    return sqrt(squared)


def segment_box_distance(start: Vector3, end: Vector3, box: AxisAlignedBox) -> float:
    """Return the exact minimum Euclidean distance from a segment to an AABB."""
    direction = tuple(b - a for a, b in zip(start, end))
    breakpoints = {0.0, 1.0}
    for origin, delta, minimum, maximum in zip(start, direction, box.minimum, box.maximum):
        if delta != 0.0:
            for boundary in (minimum, maximum):
                value = (boundary - origin) / delta
                if 0.0 < value < 1.0:
                    breakpoints.add(value)
    ordered = sorted(breakpoints)
    candidates = set(ordered)
    for lower, upper in zip(ordered, ordered[1:]):
        middle = (lower + upper) / 2.0
        numerator = 0.0
        denominator = 0.0
        for origin, delta, minimum, maximum in zip(start, direction, box.minimum, box.maximum):
            middle_value = origin + delta * middle
            active_boundary: float | None = minimum if middle_value < minimum else maximum if middle_value > maximum else None
            if active_boundary is not None:
                numerator += delta * (active_boundary - origin)
                denominator += delta * delta
        if denominator > 0.0:
            candidates.add(min(upper, max(lower, numerator / denominator)))
    return min(
        point_box_distance(
            tuple(origin + delta * value for origin, delta in zip(start, direction)),  # type: ignore[arg-type]
            box,
        )
        for value in candidates
    )


def validate_indoor_config(config: IndoorReferenceConfig) -> None:
    if config.schema_version != "echorescue-indoor-reference-stack/1.0" or config.milestone != "v0.14.5":
        raise ValueError("indoor configuration must declare the v0.14.5 schema")
    names = [entity.name for entity in config.entities]
    topics = [entity.contact_topic for entity in config.entities]
    target_ids = [target.target_id for target in config.targets]
    if len(names) != len(set(names)) or len(topics) != len(set(topics)) or len(target_ids) != len(set(target_ids)):
        raise ValueError("entity names, contact topics, and target IDs must be unique")
    if sum(entity.classification == "obstacle" for entity in config.entities) != 1:
        raise ValueError("indoor reference world requires exactly one obstacle")
    positive = (
        config.doorway.width_m, config.doorway.height_m, config.doorway.crossing_hysteresis_m,
        config.vehicle_radius_m, config.safety_margin_m, config.required_center_clearance_m,
        config.launch_ground_contact_radius_m, config.ground_contact_maximum_center_z_m,
        config.takeoff_altitude_m, config.sample_interval_s, config.return_region_radius_m,
        config.agreement_tolerance_m,
    )
    if not all(isfinite(value) and value > 0.0 for value in positive):
        raise ValueError("indoor tolerances and dimensions must be finite and positive")
    expected_clearance = config.vehicle_radius_m + config.safety_margin_m
    if abs(config.required_center_clearance_m - expected_clearance) > 1e-9:
        raise ValueError("required center clearance must equal vehicle radius plus safety margin")
    if config.doorway.width_m <= 2.0 * expected_clearance or config.doorway.height_m <= 2.0 * expected_clearance:
        raise ValueError("doorway is not traversable by the conservative vehicle envelope")
    if any(low >= high for low, high in zip(config.allowed_minimum, config.allowed_maximum)):
        raise ValueError("allowed flight volume is invalid")
    route = [(config.launch_world_enu[0], config.launch_world_enu[1], config.launch_world_enu[2] + config.takeoff_altitude_m)]
    route.extend(target.absolute(config.launch_world_enu) for target in config.targets)
    for point in route:
        if any(value < low or value > high for value, low, high in zip(point, config.allowed_minimum, config.allowed_maximum)):
            raise ValueError(f"reference point lies outside allowed flight volume: {point}")
    for start, end in zip(route, route[1:]):
        for entity in config.entities:
            clearance = segment_box_distance(start, end, entity)
            if clearance + 1e-9 < config.required_center_clearance_m:
                raise ValueError(
                    f"reference segment {start}->{end} clearance to {entity.name} is {clearance:.3f} m"
                )
    if len(config.targets) < 5:
        raise ValueError("indoor reference mission requires the complete ordered route")
    far_targets = [target for target in config.targets if target.target_id == "far-room"]
    if len(far_targets) != 1:
        raise ValueError("indoor reference mission requires exactly one far-room target")
    direct_goal = far_targets[0].absolute(config.launch_world_enu)
    if segment_box_distance(route[0], direct_goal, config.obstacle) >= config.required_center_clearance_m:
        raise ValueError("blocking obstacle does not invalidate the direct launch-to-goal segment")


def _message_time(message: Mapping[str, Any]) -> float | None:
    header = message.get("header")
    stamps = header.get("stamp", []) if isinstance(header, Mapping) else []
    if isinstance(stamps, Mapping):
        stamps = [stamps]
    if isinstance(stamps, Sequence) and stamps:
        stamp = stamps[0]
        if isinstance(stamp, Mapping):
            return float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) / 1e9
    return None


def parse_pose_message(message: Mapping[str, Any], model_name: str) -> tuple[float | None, Vector3] | None:
    poses = message.get("pose", [])
    if isinstance(poses, Mapping):
        poses = [poses]
    if not isinstance(poses, Sequence):
        return None
    for pose in poses:
        if not isinstance(pose, Mapping) or pose.get("name") != model_name:
            continue
        position = pose.get("position")
        if not isinstance(position, Mapping):
            return None
        point = (float(position["x"]), float(position["y"]), float(position["z"]))
        return _message_time(message), point
    return None


def parse_contact_message(message: Mapping[str, Any]) -> tuple[float | None, list[tuple[str, str]]]:
    contacts = message.get("contact", [])
    if isinstance(contacts, Mapping):
        contacts = [contacts]
    parsed: list[tuple[str, str]] = []
    if isinstance(contacts, Sequence):
        for contact in contacts:
            if isinstance(contact, Mapping):
                first, second = contact.get("collision1", ""), contact.get("collision2", "")
                first_name = str(first.get("name", "")) if isinstance(first, Mapping) else str(first)
                second_name = str(second.get("name", "")) if isinstance(second, Mapping) else str(second)
                parsed.append((first_name, second_name))
    return _message_time(message), parsed


class IndoorRunEvaluator:
    """Accumulate one-way Gazebo evidence without producing control inputs."""

    def __init__(self, config: IndoorReferenceConfig) -> None:
        self.config = config
        self.trajectory: list[dict[str, float]] = []
        self.contact_topics_available: set[str] = set()
        self.contact_summary: dict[str, dict[str, Any]] = {}
        self.prohibited_contacts: list[dict[str, Any]] = []
        self.doorway_crossings: list[dict[str, Any]] = []
        self.far_side_entered = False
        self.return_region_entered = False
        self._last_sample_time: float | None = None
        self._last_stable_side = 0
        self._last_stable_position: Vector3 | None = None
        self._airborne_seen = False
        self.minimum_obstacle_center_distance_m = float("inf")
        self.minimum_doorway_boundary_center_distance_m = float("inf")

    def set_available_topics(self, topics: Sequence[str]) -> None:
        self.contact_topics_available = set(topics) & set(self.config.contact_topics)

    def _floor_contact_allowed(self, position: Vector3) -> bool:
        horizontal = sqrt(
            (position[0] - self.config.launch_world_enu[0]) ** 2
            + (position[1] - self.config.launch_world_enu[1]) ** 2
        )
        return (
            horizontal <= self.config.launch_ground_contact_radius_m
            and position[2] <= self.config.ground_contact_maximum_center_z_m
        )

    def observe_pose(self, position: Vector3, simulation_time_s: float) -> None:
        if self._last_sample_time is not None and simulation_time_s - self._last_sample_time < self.config.sample_interval_s:
            return
        self._last_sample_time = simulation_time_s
        self.trajectory.append({
            "simulation_time_s": simulation_time_s,
            "east_m": position[0], "north_m": position[1], "up_m": position[2],
        })
        if position[2] > self.config.ground_contact_maximum_center_z_m + self.config.vehicle_radius_m:
            self._airborne_seen = True
        obstacle_distance = point_box_distance(position, self.config.obstacle)
        self.minimum_obstacle_center_distance_m = min(self.minimum_obstacle_center_distance_m, obstacle_distance)
        for entity in self.config.entities:
            if point_box_distance(position, entity) <= self.config.vehicle_radius_m:
                allowed = entity.classification == "floor" and self._floor_contact_allowed(position)
                if not allowed:
                    evidence = {
                        "entity": entity.name,
                        "classification": entity.classification,
                        "simulation_time_s": simulation_time_s,
                        "source": "conservative_vehicle_envelope",
                    }
                    if not self.prohibited_contacts or self.prohibited_contacts[-1] != evidence:
                        self.prohibited_contacts.append(evidence)
        self._observe_doorway(position, simulation_time_s)
        if position[0] >= self.config.far_side_minimum_x_m:
            self.far_side_entered = True
        if self._airborne_seen and self.doorway_crossings:
            horizontal = sqrt(
                (position[0] - self.config.launch_world_enu[0]) ** 2
                + (position[1] - self.config.launch_world_enu[1]) ** 2
            )
            if horizontal <= self.config.return_region_radius_m:
                self.return_region_entered = True

    def _observe_doorway(self, position: Vector3, simulation_time_s: float) -> None:
        offset = position[0] - self.config.doorway.plane_coordinate_m
        hysteresis = self.config.doorway.crossing_hysteresis_m
        side = -1 if offset <= -hysteresis else 1 if offset >= hysteresis else 0
        if side == 0:
            return
        if self._last_stable_side and side != self._last_stable_side and self._last_stable_position is not None:
            start = self._last_stable_position
            denominator = position[0] - start[0]
            if denominator != 0.0:
                fraction = (self.config.doorway.plane_coordinate_m - start[0]) / denominator
                y = start[1] + fraction * (position[1] - start[1])
                z = start[2] + fraction * (position[2] - start[2])
                inside = (
                    self.config.doorway.minimum_y_m <= y <= self.config.doorway.maximum_y_m
                    and self.config.doorway.bottom_m <= z <= self.config.doorway.top_m
                )
                self.doorway_crossings.append({
                    "direction": "outbound" if side > 0 else "inbound",
                    "simulation_time_s": simulation_time_s,
                    "crossing_y_m": y, "crossing_z_m": z, "inside_opening": inside,
                })
                boundary_distance = min(
                    y - self.config.doorway.minimum_y_m,
                    self.config.doorway.maximum_y_m - y,
                    z - self.config.doorway.bottom_m,
                    self.config.doorway.top_m - z,
                )
                self.minimum_doorway_boundary_center_distance_m = min(
                    self.minimum_doorway_boundary_center_distance_m, boundary_distance,
                )
        self._last_stable_side = side
        self._last_stable_position = position

    def observe_contacts(
        self, entity_name: str, collision_pairs: Sequence[tuple[str, str]],
        simulation_time_s: float, position: Vector3 | None,
    ) -> None:
        if not collision_pairs:
            return
        entity = next(item for item in self.config.entities if item.name == entity_name)
        summary = self.contact_summary.setdefault(entity_name, {
            "entity": entity_name, "classification": entity.classification,
            "first_contact_simulation_time_s": simulation_time_s,
            "last_contact_simulation_time_s": simulation_time_s,
            "contact_message_count": 0, "collision_pairs": [], "prohibited": False,
        })
        summary["last_contact_simulation_time_s"] = simulation_time_s
        summary["contact_message_count"] += 1
        for pair in collision_pairs:
            rendered = list(pair)
            if rendered not in summary["collision_pairs"]:
                summary["collision_pairs"].append(rendered)
        allowed = entity.classification == "floor" and position is not None and self._floor_contact_allowed(position)
        if not allowed:
            summary["prohibited"] = True
            self.prohibited_contacts.append({
                "entity": entity_name, "classification": entity.classification,
                "simulation_time_s": simulation_time_s, "source": "gazebo_contact_sensor",
            })

    @property
    def collision_detected(self) -> bool:
        return bool(self.prohibited_contacts)

    def report(self) -> dict[str, Any]:
        crossings = [item["direction"] for item in self.doorway_crossings if item["inside_opening"]]
        expected_crossings = ["outbound", "inbound"]
        topics_complete = set(self.config.contact_topics) <= self.contact_topics_available
        passed = (
            bool(self.trajectory) and topics_complete and not self.collision_detected
            and crossings == expected_crossings
            and self.far_side_entered and self.return_region_entered
        )
        summaries = []
        for summary in self.contact_summary.values():
            item = dict(summary)
            item["contact_duration_s"] = max(
                0.0,
                float(item["last_contact_simulation_time_s"]) - float(item["first_contact_simulation_time_s"]),
            )
            summaries.append(item)
        minimum = self.minimum_obstacle_center_distance_m
        doorway_minimum = self.minimum_doorway_boundary_center_distance_m
        reported_trajectory = self.trajectory[::10]
        if self.trajectory and reported_trajectory[-1] is not self.trajectory[-1]:
            reported_trajectory.append(self.trajectory[-1])
        return {
            "schema_version": "echorescue-indoor-gazebo-evaluation/1.0",
            "milestone": "v0.14.5",
            "status": "PASS" if passed else "FAIL",
            "world_version": self.config.world_version,
            "world_name": self.config.world_name,
            "model_name": self.config.model_name,
            "trajectory": reported_trajectory,
            "trajectory_sample_count": len(self.trajectory),
            "trajectory_reported_sample_count": len(reported_trajectory),
            "contact_topics_required": list(self.config.contact_topics),
            "contact_topics_available": sorted(self.contact_topics_available),
            "contact_topic_coverage_complete": topics_complete,
            "contacts": sorted(summaries, key=lambda item: item["entity"]),
            "contact_count": sum(int(item["contact_message_count"]) for item in summaries),
            "prohibited_contact_detected": self.collision_detected,
            "prohibited_contacts": self.prohibited_contacts,
            "doorway_crossings": self.doorway_crossings,
            "required_doorway_crossings": expected_crossings,
            "far_side_region_entered": self.far_side_entered,
            "return_region_entered": self.return_region_entered,
            "minimum_obstacle_center_distance_m": None if minimum == float("inf") else minimum,
            "minimum_obstacle_surface_clearance_m": None if minimum == float("inf") else minimum - self.config.vehicle_radius_m,
            "minimum_doorway_boundary_center_distance_m": None if doorway_minimum == float("inf") else doorway_minimum,
            "minimum_doorway_boundary_surface_clearance_m": None if doorway_minimum == float("inf") else doorway_minimum - self.config.vehicle_radius_m,
            "conservative_vehicle_radius_m": self.config.vehicle_radius_m,
            "safety_margin_m": self.config.safety_margin_m,
            "clearance_method": "sampled model origin to configured axis-aligned obstacle, minus conservative spherical vehicle radius",
            "clearance_uncertainty": "sampled at the configured interval; contact sensors independently cover every configured solid",
            "floor_contact_uncertainty": "the Gazebo floor sensor may retain its initial collision pair; only its first time-correlated launch contact is counted, while the sampled conservative envelope classifies later floor intersection",
        }


def serialize_indoor_report(report: Mapping[str, Any]) -> str:
    return json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True) + "\n"
