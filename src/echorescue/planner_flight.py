"""Known-map adapter from the existing deterministic A* to ENU targets."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from math import ceil, floor, hypot, isfinite, sqrt
from pathlib import Path
from typing import Any, Mapping, Sequence

from echorescue.models import Position
from echorescue.planning import astar


Cell = tuple[int, int]  # row, column
Point2 = tuple[float, float]  # east, north


@dataclass(frozen=True, slots=True)
class GridTransform:
    rows: int
    columns: int
    resolution_m: float
    origin_east_m: float
    origin_north_m: float
    column_axis: str
    row_axis: str

    def __post_init__(self) -> None:
        if self.rows <= 0 or self.columns <= 0 or not isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("grid dimensions and resolution must be positive")
        if self.column_axis != "east_positive" or self.row_axis != "north_positive":
            raise ValueError("v0.15.0 requires columns east-positive and rows north-positive")

    def cell_center_to_enu(self, cell: Cell) -> Point2:
        row, column = cell
        if not self.contains(cell):
            raise ValueError(f"cell outside map: {cell}")
        return (
            self.origin_east_m + (column + 0.5) * self.resolution_m,
            self.origin_north_m + (row + 0.5) * self.resolution_m,
        )

    def enu_to_cell(self, point: Point2) -> Cell:
        east, north = point
        if not all(isfinite(value) for value in point):
            raise ValueError("ENU point must be finite")
        cell = (
            floor((north - self.origin_north_m) / self.resolution_m),
            floor((east - self.origin_east_m) / self.resolution_m),
        )
        if not self.contains(cell):
            raise ValueError(f"ENU point outside map: {point}")
        return cell

    def contains(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.rows and 0 <= cell[1] < self.columns


@dataclass(frozen=True, slots=True)
class KnownMapPlan:
    map_version: str
    transform: GridTransform
    occupied: frozenset[Cell]
    inflated_occupied: frozenset[Cell]
    start: Cell
    goal: Cell
    cruise_altitude_m: float
    footprint_m: float
    raw_outbound: tuple[Cell, ...]
    compacted_outbound: tuple[Cell, ...]
    raw_return: tuple[Cell, ...]
    compacted_return: tuple[Cell, ...]
    targets: tuple[dict[str, Any], ...]

    def report(self) -> dict[str, Any]:
        return {
            "schema_version": "echorescue-known-map-plan/1.0",
            "milestone": "v0.15.0",
            "status": "PASS",
            "failure_reason": None,
            "map_version": self.map_version,
            "map_rows": self.transform.rows,
            "map_columns": self.transform.columns,
            "resolution_m_per_cell": self.transform.resolution_m,
            "transform": asdict(self.transform),
            "observed_launch_local_enu": None,
            "start_cell": list(self.start),
            "outbound_goal_cell": list(self.goal),
            "inflated_occupied_cell_count": len(self.inflated_occupied),
            "conservative_footprint_m": self.footprint_m,
            "raw_outbound_grid_path": [list(cell) for cell in self.raw_outbound],
            "compacted_outbound_grid_path": [list(cell) for cell in self.compacted_outbound],
            "raw_return_grid_path": [list(cell) for cell in self.raw_return],
            "compacted_return_grid_path": [list(cell) for cell in self.compacted_return],
            "outbound_cost": len(self.raw_outbound) - 1,
            "return_cost": len(self.raw_return) - 1,
            "generated_enu_targets": list(self.targets),
            "planner": "echorescue.planning.astar",
            "connectivity": "cardinal_4",
            "corner_cutting": False,
            "compaction": "axis_aligned_collinear_checked",
        }


def _cell(value: Sequence[Any], name: str) -> Cell:
    if len(value) != 2 or any(isinstance(item, bool) or not isinstance(item, int) for item in value):
        raise ValueError(f"{name} must be [row, column] integers")
    return int(value[0]), int(value[1])


def inflate_occupied(
    occupied: frozenset[Cell], transform: GridTransform, footprint_m: float,
) -> frozenset[Cell]:
    if not isfinite(footprint_m) or footprint_m <= 0:
        raise ValueError("conservative footprint must be positive")
    # An occupied cell represents its full square, not only its centre. Adding
    # the half-diagonal prevents under-inflation at cell corners.
    effective_radius = footprint_m + transform.resolution_m / sqrt(2.0)
    radius_cells = ceil(effective_radius / transform.resolution_m)
    inflated: set[Cell] = set()
    for row, column in occupied:
        for row_delta in range(-radius_cells, radius_cells + 1):
            for column_delta in range(-radius_cells, radius_cells + 1):
                candidate = row + row_delta, column + column_delta
                if (
                    transform.contains(candidate)
                    and hypot(row_delta, column_delta) * transform.resolution_m <= effective_radius + 1e-12
                ):
                    inflated.add(candidate)
    return frozenset(inflated)


def _to_position(cell: Cell) -> Position:
    return Position(cell[1], cell[0])


def _from_position(position: Position) -> Cell:
    return position.y, position.x


def _path(start: Cell, goal: Cell, transform: GridTransform, blocked: frozenset[Cell]) -> tuple[Cell, ...]:
    result = astar(
        _to_position(start), _to_position(goal),
        lambda position: transform.contains(_from_position(position)) and _from_position(position) not in blocked,
    )
    if result is None:
        raise ValueError(f"no deterministic path from {start} to {goal} after inflation")
    converted = tuple(_from_position(position) for position in result)
    if not converted or converted[0] != start or converted[-1] != goal:
        raise ValueError("existing A* returned malformed path")
    return converted


def segment_cells(start: Cell, end: Cell) -> tuple[Cell, ...]:
    if start[0] != end[0] and start[1] != end[1]:
        raise ValueError("v0.15.0 compaction permits axis-aligned segments only")
    row_step = 0 if start[0] == end[0] else (1 if end[0] > start[0] else -1)
    column_step = 0 if start[1] == end[1] else (1 if end[1] > start[1] else -1)
    length = max(abs(end[0] - start[0]), abs(end[1] - start[1]))
    return tuple((start[0] + index * row_step, start[1] + index * column_step) for index in range(length + 1))


def compact_path(path: tuple[Cell, ...], blocked: frozenset[Cell]) -> tuple[Cell, ...]:
    if not path:
        raise ValueError("cannot compact an empty path")
    compacted = [path[0]]
    for index in range(1, len(path) - 1):
        previous, current, following = path[index - 1], path[index], path[index + 1]
        first_direction = current[0] - previous[0], current[1] - previous[1]
        second_direction = following[0] - current[0], following[1] - current[1]
        if first_direction != second_direction:
            compacted.append(current)
    compacted.append(path[-1])
    for start, end in zip(compacted, compacted[1:]):
        if any(cell in blocked for cell in segment_cells(start, end)):
            raise ValueError(f"compacted segment intersects inflated occupancy: {start}->{end}")
    return tuple(compacted)


def _targets(
    outbound: tuple[Cell, ...], returning: tuple[Cell, ...], transform: GridTransform,
    altitude_m: float,
) -> tuple[dict[str, Any], ...]:
    generated: list[dict[str, Any]] = []
    for index, cell in enumerate(outbound):
        target_id = "launch-stabilize" if index == 0 else "outbound-goal" if index == len(outbound) - 1 else f"outbound-{index:03d}"
        east, north = transform.cell_center_to_enu(cell)
        generated.append({"target_id": target_id, "east_offset_m": east, "north_offset_m": north, "altitude_above_launch_m": altitude_m, "source_cell": list(cell), "leg": "outbound"})
    for index, cell in enumerate(returning[1:], 1):
        target_id = "return-launch" if index == len(returning) - 1 else f"return-{index:03d}"
        east, north = transform.cell_center_to_enu(cell)
        generated.append({"target_id": target_id, "east_offset_m": east, "north_offset_m": north, "altitude_above_launch_m": altitude_m, "source_cell": list(cell), "leg": "return"})
    return tuple(generated)


def plan_known_map(config: Mapping[str, Any]) -> KnownMapPlan:
    if config.get("schema_version") != "echorescue-planner-flight-stack/1.0" or config.get("milestone") != "v0.15.0":
        raise ValueError("planner-flight configuration must declare v0.15.0 schema")
    map_config, planning = config["map"], config["planning"]
    transform = GridTransform(
        int(map_config["rows"]), int(map_config["columns"]), float(map_config["resolution_m_per_cell"]),
        float(map_config["origin_relative_to_launch_enu_m"][0]), float(map_config["origin_relative_to_launch_enu_m"][1]),
        str(map_config["column_axis"]), str(map_config["row_axis"]),
    )
    rows = map_config["occupancy_rows"]
    if not isinstance(rows, list) or len(rows) != transform.rows or any(not isinstance(row, str) or len(row) != transform.columns for row in rows):
        raise ValueError("occupancy rows do not match configured dimensions")
    if any(character not in ".#" for row in rows for character in row):
        raise ValueError("known map supports only free '.' and occupied '#' cells")
    if map_config.get("row_storage_order") != "south_to_north":
        raise ValueError("occupancy rows must be stored south-to-north")
    if planning.get("connectivity") != "cardinal_4" or planning.get("allow_diagonal_corner_cutting") is not False:
        raise ValueError("v0.15.0 reuses cardinal A* and forbids diagonal corner cutting")
    if planning.get("path_compaction") != "axis_aligned_collinear_checked":
        raise ValueError("unsupported path compaction policy")
    if planning.get("start_cell_policy") != "observed_launch_anchor_cell":
        raise ValueError("v0.15.0 requires the observed-launch start-cell policy")
    if planning.get("return_goal_policy") != "recorded_launch_cell":
        raise ValueError("v0.15.0 requires return to the recorded launch cell")
    start, goal = _cell(planning["start_cell"], "start cell"), _cell(planning["outbound_goal_cell"], "goal cell")
    if not transform.contains(start):
        raise ValueError("configured start cell is outside the map")
    if not transform.contains(goal):
        raise ValueError("configured outbound goal cell is outside the map")
    if transform.enu_to_cell((0.0, 0.0)) != start:
        raise ValueError("configured start cell is inconsistent with the launch-relative transform")
    occupied = frozenset((row, column) for row, value in enumerate(rows) for column, character in enumerate(value) if character == "#")
    vehicle_radius = float(planning["conservative_vehicle_radius_m"])
    safety_margin = float(planning["safety_margin_m"])
    if not isfinite(vehicle_radius) or vehicle_radius <= 0:
        raise ValueError("conservative vehicle radius must be positive")
    if not isfinite(safety_margin) or safety_margin < 0:
        raise ValueError("safety margin must be non-negative")
    footprint = vehicle_radius + safety_margin
    inflated = inflate_occupied(occupied, transform, footprint)
    if start in inflated:
        raise ValueError("launch start cell is occupied after inflation")
    if goal in inflated:
        raise ValueError("outbound goal cell is occupied after inflation")
    outbound = _path(start, goal, transform, inflated)
    returning = _path(goal, start, transform, inflated)
    compacted_outbound = compact_path(outbound, inflated)
    compacted_return = compact_path(returning, inflated)
    altitude = float(planning["cruise_altitude_above_launch_m"])
    if not isfinite(altitude) or altitude <= 0:
        raise ValueError("cruise altitude must be positive")
    targets = _targets(compacted_outbound, compacted_return, transform, altitude)
    if not targets:
        raise ValueError("planner generated no executable targets")
    flight = config["flight"]
    if not (
        float(flight["geofence_min_altitude_m"]) <= altitude
        <= float(flight["geofence_max_altitude_m"])
    ):
        raise ValueError("cruise altitude violates the configured vertical geofence")
    horizontal_limit = float(flight["geofence_horizontal_radius_m"])
    if any(hypot(float(item["east_offset_m"]), float(item["north_offset_m"])) > horizontal_limit for item in targets):
        raise ValueError("generated route violates the configured horizontal geofence")
    return KnownMapPlan(
        str(config["map_version"]), transform, occupied, inflated, start, goal,
        altitude, footprint, outbound, compacted_outbound, returning,
        compacted_return, targets,
    )


def load_planner_flight_config(path: Path | str) -> dict[str, Any]:
    config: dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
    plan_known_map(config)
    return config


def serialize_planning_report(report: Mapping[str, Any]) -> str:
    return json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True) + "\n"
