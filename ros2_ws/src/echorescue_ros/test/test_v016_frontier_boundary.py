"""Native Jazzy interface and trust-boundary checks for v0.16.0."""

from inspect import getsource

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("echorescue_interfaces")

from echorescue_interfaces.msg import ExplorationState
from echorescue_ros.frontier_exploration_observer import FrontierExplorationObserver
from echorescue_ros.mavlink_frontier_exploration import MavlinkFrontierExploration
from echorescue_ros.mavlink_telemetry_bridge import MavlinkTelemetryBridge
from echorescue_ros.qos import MISSION_QOS


def test_exploration_message_schema_has_compact_reconstruction_fields() -> None:
    fields = ExplorationState.get_fields_and_field_types()
    required = {
        "schema_version", "session_id", "sequence", "state", "event",
        "map_revision", "unknown_cell_count", "free_cell_count",
        "occupied_cell_count", "inflated_cell_count", "accepted_scan_count",
        "rejected_scan_count", "frontier_cluster_count",
        "has_selected_frontier", "route_generation", "active_target_id",
    }
    assert required <= fields.keys()


def test_observer_is_typed_ros_only() -> None:
    source = getsource(FrontierExplorationObserver)
    for prohibited in ("pymavlink", "gz.msgs", "/world/", "subprocess"):
        assert prohibited not in source


def test_production_explorer_has_no_gazebo_ground_truth_access() -> None:
    source = getsource(MavlinkFrontierExploration)
    for prohibited in ("gz.msgs", "/world/", "contact", "model pose", "evaluator"):
        assert prohibited not in source.lower()


def test_legacy_bridge_remains_receive_only_and_qos_is_bounded() -> None:
    assert "command_long_send" not in getsource(MavlinkTelemetryBridge)
    assert MISSION_QOS.depth > 0


# --- Deterministic frontier-unreachable -> conservative-return transition ---
#
# These tests instantiate the production node directly (no Gazebo, no
# spinning, no MAVLink transport) and drive `_invalidate_unsafe_route`
# through white-box map/controller state, matching the existing
# `tests/test_waypoint_mission.py` pattern of direct controller-state setup.

from pathlib import Path
from time import monotonic_ns

import rclpy

from echorescue.frontier_exploration import ExplorationState as _ExplorationEnum
from echorescue.models import CellState
from echorescue.planner_flight import inflate_occupied
from echorescue.sensor_replanning import PoseSample
from echorescue.waypoint_mission import NavigationPhase

_CONFIG_PATH = Path(__file__).resolve().parents[4] / "config" / "frontier-exploration-v0.16.0.json"
_VISUAL_CONFIG_PATH = (
    Path(__file__).resolve().parents[4] / "config" / "visual-frontier-exploration-v0.16.1.json"
)


@pytest.fixture(scope="module", autouse=True)
def _rclpy_context():
    if not rclpy.ok():
        rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def visual_horizon_node(tmp_path):
    pytest.importorskip("pymavlink")
    node = MavlinkFrontierExploration(
        tmp_path / "horizon-artifact.json", _VISUAL_CONFIG_PATH,
    )
    yield node
    node.destroy_node()


def _wall_partitioned_node(tmp_path, *, leave_unknown_patch: bool) -> MavlinkFrontierExploration:
    """Build a fully-mapped grid split by one occupied wall, current on one side."""
    # Node construction loads pymavlink through the transport bridge, even
    # though these deterministic tests never connect to MAVLink.
    pytest.importorskip("pymavlink")
    node = MavlinkFrontierExploration(tmp_path / "artifact.json", _CONFIG_PATH)
    node.controller.flight.session_id = "s1"
    node.controller.launch_enu = (0.0, 0.0, 0.0)
    transform = node.map.policy.transform
    for row in range(transform.rows):
        for column in range(transform.columns):
            node.map._states[node.map._index((row, column))] = CellState.FREE
    if leave_unknown_patch:
        for row in range(transform.rows - 4, transform.rows):
            for column in range(transform.columns - 4, transform.columns):
                node.map._states[node.map._index((row, column))] = CellState.UNKNOWN
    wall_column = 15
    wall_cells = frozenset((row, wall_column) for row in range(transform.rows))
    for cell in wall_cells:
        node.map._states[node.map._index(cell)] = CellState.OCCUPIED
    node.map._occupied = set(wall_cells)
    node.map._inflated = inflate_occupied(wall_cells, transform, node.map.policy.footprint_m)

    current_cell = (transform.rows // 2, transform.columns - 5)
    launch_cell = transform.enu_to_cell((0.0, 0.0))
    assert launch_cell[1] < wall_column - 3, "launch must sit clear of the wall on the near side"
    assert current_cell[1] > wall_column + 3, "current cell must sit clear of the wall on the far side"
    assert node.map.safe_free(current_cell)
    assert node.map.safe_free(launch_cell)

    active_frontier = (launch_cell[0], max(0, launch_cell[1] - 2))
    assert node.map.safe_free(active_frontier)
    assert node.map.path(current_cell, active_frontier) is None, "wall must sever the active frontier"
    assert node.map.path(current_cell, launch_cell) is None, "wall must sever the return route too"

    east, north = transform.cell_center_to_enu(current_cell)
    node.map.observe_pose(PoseSample("s1", 100, monotonic_ns(), east, north, 0.0, 0.0, 0.0))
    node.active_frontier = active_frontier
    node.active_route = (current_cell, (current_cell[0], wall_column))
    node.controller.phase = NavigationPhase.TRACK_TARGET
    return node


def _frontier_severed_but_return_known_node(tmp_path) -> MavlinkFrontierExploration:
    """Fully-mapped grid where the wall only severs the active frontier.

    Current and launch stay on the same (near) side of the wall, so a
    known-safe return path exists even though the active frontier (on the
    far side) becomes unreachable and no other frontier candidates remain
    (the grid has no UNKNOWN cells left to explore).
    """
    pytest.importorskip("pymavlink")
    node = MavlinkFrontierExploration(tmp_path / "artifact.json", _CONFIG_PATH)
    node.controller.flight.session_id = "s1"
    node.controller.launch_enu = (0.0, 0.0, 0.0)
    transform = node.map.policy.transform
    for row in range(transform.rows):
        for column in range(transform.columns):
            node.map._states[node.map._index((row, column))] = CellState.FREE
    wall_column = 15
    wall_cells = frozenset((row, wall_column) for row in range(transform.rows))
    for cell in wall_cells:
        node.map._states[node.map._index(cell)] = CellState.OCCUPIED
    node.map._occupied = set(wall_cells)
    node.map._inflated = inflate_occupied(wall_cells, transform, node.map.policy.footprint_m)

    launch_cell = transform.enu_to_cell((0.0, 0.0))
    assert launch_cell[1] < wall_column - 3, "launch must sit clear of the wall on the near side"
    current_cell = (launch_cell[0], max(0, launch_cell[1] - 3))
    assert current_cell[1] < wall_column - 3, "current cell must stay on launch's side of the wall"
    active_frontier = (launch_cell[0], wall_column + 5)
    assert node.map.safe_free(current_cell)
    assert node.map.safe_free(launch_cell)
    assert node.map.safe_free(active_frontier)

    return_path = node.map.path(current_cell, launch_cell)
    assert return_path is not None and len(return_path) >= 2, (
        "current and launch must share a known-safe route on the near side of the wall"
    )
    assert node.map.path(current_cell, active_frontier) is None, "wall must sever the active frontier"

    east, north = transform.cell_center_to_enu(current_cell)
    node.map.observe_pose(PoseSample("s1", 100, monotonic_ns(), east, north, 0.0, 0.0, 0.0))
    node.active_frontier = active_frontier
    node.active_route = (current_cell, (current_cell[0], wall_column))
    node.controller.phase = NavigationPhase.TRACK_TARGET
    return node


def test_unreachable_active_frontier_without_alternatives_holds_instead_of_failing(tmp_path) -> None:
    node = _wall_partitioned_node(tmp_path, leave_unknown_patch=False)

    node._invalidate_unsafe_route()

    assert node.exploration_failure is None, (
        "an active frontier severed by new occupancy must not be treated as a "
        "hard mission failure while the map has no other pending frontier"
    )
    assert node.returning is False
    assert node.state is _ExplorationEnum.REPLANNING
    assert node.active_frontier is None
    assert node.active_route and all(cell == node.active_route[0] for cell in node.active_route)
    assert not node.controller.terminal


def test_unreachable_active_frontier_with_alternative_resumes_exploration(tmp_path) -> None:
    node = _wall_partitioned_node(tmp_path, leave_unknown_patch=True)
    severed_frontier = node.active_frontier

    node._invalidate_unsafe_route()

    assert node.exploration_failure is None
    assert node.returning is False
    assert node.state is not _ExplorationEnum.RECOVERY
    assert severed_frontier in node.visited_frontiers
    assert node.active_frontier is not None
    assert node.active_frontier != severed_frontier
    assert node.active_route and node.active_route[-1] != severed_frontier
    assert not node.controller.terminal


def _unsafe_current_cell_node(tmp_path) -> MavlinkFrontierExploration:
    """Fully-mapped grid, wall severs both frontier and return, and the
    vehicle's own current cell is itself inflated (unsafe) at invalidation
    time -- for example, new occupancy inflation has just swept over it.
    Neither a hold nor a return may be issued from an unsafe current cell.
    """
    pytest.importorskip("pymavlink")
    node = MavlinkFrontierExploration(tmp_path / "artifact.json", _CONFIG_PATH)
    node.controller.flight.session_id = "s1"
    node.controller.launch_enu = (0.0, 0.0, 0.0)
    transform = node.map.policy.transform
    for row in range(transform.rows):
        for column in range(transform.columns):
            node.map._states[node.map._index((row, column))] = CellState.FREE
    wall_column = 15
    wall_cells = frozenset((row, wall_column) for row in range(transform.rows))
    for cell in wall_cells:
        node.map._states[node.map._index(cell)] = CellState.OCCUPIED
    node.map._occupied = set(wall_cells)
    node.map._inflated = inflate_occupied(wall_cells, transform, node.map.policy.footprint_m)

    launch_cell = transform.enu_to_cell((0.0, 0.0))
    assert launch_cell[1] < wall_column - 3, "launch must sit clear of the wall on the near side"
    # Immediately adjacent to the wall: guaranteed to fall inside its
    # inflation footprint regardless of the configured conservative radius.
    current_cell = (transform.rows // 2, wall_column + 1)
    assert current_cell in node.map.inflated, "current cell must be unsafe (inflated) for this scenario"
    assert not node.map.safe_free(current_cell)

    active_frontier = (launch_cell[0], max(0, launch_cell[1] - 2))
    assert node.map.safe_free(active_frontier)
    assert node.map.path(current_cell, active_frontier) is None, "wall/inflation must sever the active frontier"
    assert node.map.path(current_cell, launch_cell) is None, "wall/inflation must sever the return route too"

    east, north = transform.cell_center_to_enu(current_cell)
    node.map.observe_pose(PoseSample("s1", 100, monotonic_ns(), east, north, 0.0, 0.0, 0.0))
    node.active_frontier = active_frontier
    node.active_route = (current_cell, (current_cell[0], wall_column))
    node.controller.phase = NavigationPhase.TRACK_TARGET
    return node


def test_unreachable_active_frontier_with_known_safe_route_commits_to_return(tmp_path) -> None:
    node = _frontier_severed_but_return_known_node(tmp_path)
    launch_cell = node.map.policy.transform.enu_to_cell((0.0, 0.0))
    severed_frontier = node.active_frontier

    node._invalidate_unsafe_route()

    assert node.exploration_failure is None, (
        "a known-safe route home must be used instead of failing the mission"
    )
    assert node.returning is True
    assert node.state is _ExplorationEnum.RETURNING
    assert node.active_frontier is None
    assert node.active_route, "a concrete return route must be installed"
    assert all(node.map.safe_free(cell) for cell in node.active_route), (
        "every commanded return cell must be known-safe"
    )
    assert node.active_route[-1] != severed_frontier
    assert node.active_route[-1][1] <= launch_cell[1], (
        "the commanded horizon must progress toward launch, not toward the severed frontier"
    )
    assert not node.controller.terminal


def test_unreachable_active_frontier_with_unsafe_current_cell_fails_recovery_instead_of_holding(
    tmp_path,
) -> None:
    node = _unsafe_current_cell_node(tmp_path)
    unsafe_current = node._current_cell()
    assert unsafe_current is not None and not node.map.safe_free(unsafe_current)

    node._invalidate_unsafe_route()

    assert node.exploration_failure is not None, (
        "an unsafe current cell must not silently resolve into a hold or a "
        "return commitment; the existing failure/recovery policy must run"
    )
    assert node.state is _ExplorationEnum.RECOVERY
    assert node.returning is False, (
        "no return may be committed from an unsafe current cell"
    )
    # The known-safe route contract must not be violated: no new hold/return
    # target may have been installed at (or from) the unsafe current cell.
    assert node.active_route != (unsafe_current,)
    assert node.controller.terminal or node.controller.phase is not NavigationPhase.TRACK_TARGET


def test_visual_command_horizon_is_at_most_half_meter_and_advances_one_cell(
    visual_horizon_node: MavlinkFrontierExploration,
) -> None:
    transform = visual_horizon_node.map.policy.transform
    path = tuple((transform.rows // 2, column) for column in range(transform.columns))

    assert visual_horizon_node.raw["flight"]["command_horizon_m"] == pytest.approx(0.5)
    command_path = visual_horizon_node._command_horizon(path)
    traveled_m = (len(command_path) - 1) * transform.resolution_m

    assert len(command_path) >= 2, "the horizon must include at least one path step"
    assert len(command_path) < len(path), "the long path must be truncated"
    assert traveled_m <= 0.5


def test_visual_command_horizon_honors_configured_distance(
    visual_horizon_node: MavlinkFrontierExploration,
) -> None:
    transform = visual_horizon_node.map.policy.transform
    path = tuple((transform.rows // 2, column) for column in range(transform.columns))
    configured_horizon_m = 0.75
    visual_horizon_node.raw["flight"]["command_horizon_m"] = configured_horizon_m

    command_path = visual_horizon_node._command_horizon(path)
    traveled_m = (len(command_path) - 1) * transform.resolution_m

    assert len(command_path) >= 2, "the configured horizon must include at least one path step"
    assert traveled_m == pytest.approx(configured_horizon_m)


def test_legacy_command_horizon_retains_one_meter_default(
    visual_horizon_node: MavlinkFrontierExploration,
) -> None:
    transform = visual_horizon_node.map.policy.transform
    path = tuple((transform.rows // 2, column) for column in range(transform.columns))
    visual_horizon_node.raw["flight"].pop("command_horizon_m")

    command_path = visual_horizon_node._command_horizon(path)
    traveled_m = (len(command_path) - 1) * transform.resolution_m

    assert traveled_m == pytest.approx(1.0)
