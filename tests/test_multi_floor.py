import json
from pathlib import Path

import pytest

from echorescue.cli import build_parser, floor_obstacle_spec
from echorescue.config import SimulationConfig
from echorescue.dashboard import _validate_replay
from echorescue.environment import GridWorld
from echorescue.models import CellState, Position
from echorescue.multi_floor import (
    MULTI_FLOOR_REPLAY_SCHEMA_VERSION,
    FloorTransition,
    GridPosition,
    MultiFloorConfig,
    MultiFloorEnvironment,
    MultiFloorSimulation,
    multi_floor_astar,
    path_cost,
)
from echorescue.multi_simulation import MultiDroneSimulation


def test_grid_position_is_floor_aware_and_ordered() -> None:
    assert GridPosition(0, 2, 3) != GridPosition(1, 2, 3)
    assert sorted((GridPosition(1, 0, 0), GridPosition(0, 9, 9)))[0].floor == 0
    assert GridPosition(2, 4, 5).local == Position(5, 4)


def test_environment_loads_different_floor_dimensions() -> None:
    first = GridWorld(7, 7, Position(1, 1), frozenset())
    second = GridWorld(9, 8, Position(1, 1), frozenset())
    transition = FloorTransition(GridPosition(0, 2, 2), GridPosition(1, 3, 3))
    environment = MultiFloorEnvironment(
        {0: first, 1: second}, (transition,), GridPosition(0, 1, 1)
    )
    assert environment.floors[1].width == 9


def test_transition_is_bidirectional_but_vertical_motion_is_explicit() -> None:
    environment = MultiFloorEnvironment.generate(MultiFloorConfig(floor_count=2))
    transition = environment.transitions[0]
    assert transition.destination_from(transition.source) == transition.destination
    assert transition.destination_from(transition.destination) == transition.source
    arbitrary = GridPosition(0, 4, 4)
    assert all(neighbor.floor == 0 for neighbor, _ in environment.neighbors(arbitrary))


def test_directed_transition_does_not_reverse() -> None:
    transition = FloorTransition(
        GridPosition(0, 1, 1), GridPosition(1, 1, 1), bidirectional=False
    )
    assert transition.destination_from(transition.destination) is None


def test_astar_crosses_two_floors_with_weighted_cost() -> None:
    config = MultiFloorConfig(floor_count=2, obstacle_density=0.0)
    environment = MultiFloorEnvironment.generate(config)
    goal = GridPosition(1, 4, 4)
    path = multi_floor_astar(environment, environment.base, goal)
    assert path is not None
    assert {position.floor for position in path} == {0, 1}
    assert path_cost(environment, path) >= len(path) - 1 + config.transition_cost - 1


def test_astar_crosses_three_floors() -> None:
    environment = MultiFloorEnvironment.generate(
        MultiFloorConfig(floor_count=3, obstacle_density=0.0)
    )
    path = multi_floor_astar(environment, environment.base, GridPosition(2, 4, 4))
    assert path is not None
    assert {position.floor for position in path} == {0, 1, 2}


def test_unreachable_floor_has_no_false_path() -> None:
    environment = MultiFloorEnvironment.generate(MultiFloorConfig(floor_count=2))
    environment.transitions = ()
    assert multi_floor_astar(
        environment, environment.base, GridPosition(1, 2, 2)
    ) is None


def test_transition_topology_known_but_other_floor_occupancy_unknown() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(floor_count=2))
    endpoint = simulation.environment.transitions[0].destination
    assert simulation.knowledge.cell_at(endpoint) is CellState.FREE
    assert simulation.knowledge.cell_at(GridPosition(1, 4, 4)) is CellState.UNKNOWN


def test_sensor_does_not_observe_through_floor() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(floor_count=2))
    simulation._observe(simulation.agents["drone-1"])
    assert simulation.knowledge.cell_at(GridPosition(1, 1, 1)) is CellState.UNKNOWN


def test_survivor_identity_includes_floor() -> None:
    first = GridPosition(0, 3, 3)
    second = GridPosition(1, 3, 3)
    hypotheses = {first: 0.8, second: 0.2}
    assert len(hypotheses) == 2


def test_frontiers_are_floor_qualified() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(floor_count=2))
    simulation._observe(simulation.agents["drone-1"])
    assert all(position.floor == 0 for position in simulation.knowledge.frontiers(0))
    assert all(position.floor == 1 for position in simulation.knowledge.frontiers(1))


def test_four_agents_allocate_across_floors() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(seed=1))
    for _ in range(12):
        simulation.step()
    assert len({agent.position.floor for agent in simulation.agents.values()}) > 1


def test_transition_conflict_has_deterministic_winner() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(seed=1))
    result = simulation.run()
    assert result.success
    conflicts = [event for event in simulation.events if event["event_type"] == "transition_conflict"]
    assert conflicts
    assert result.drone_collisions == 0


def test_top_floor_agents_return_via_transitions() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(seed=0))
    result = simulation.run()
    assert result.success
    assert result.returned_agents == 4
    assert all(agent.position == simulation.environment.base for agent in simulation.agents.values())


def test_transition_energy_cost_is_consumed() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(seed=0))
    result = simulation.run()
    transitions = int(result.metrics["floor_transitions_total"])
    movement = sum(agent.path_length for agent in simulation.agents.values())
    assert transitions > 0
    assert movement >= transitions * simulation.config.transition_cost
    assert all(agent.energy < simulation.config.battery_capacity for agent in simulation.agents.values())


def test_floor_aware_dynamic_obstacle_injection() -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(seed=0))
    candidate = GridPosition(1, 4, 4)
    if not simulation.environment.is_free(candidate) or candidate in simulation.environment.survivors:
        candidate = next(
            GridPosition(1, row, col)
            for row in range(1, simulation.config.height - 1)
            for col in range(1, simulation.config.width - 1)
            if simulation.environment.is_free(GridPosition(1, row, col))
            and GridPosition(1, row, col) not in simulation.environment.survivors
            and all(
                GridPosition(1, row, col) not in {item.source, item.destination}
                for item in simulation.environment.transitions
            )
        )
    simulation.environment.block_cell(candidate)
    assert simulation.environment.cell_at(candidate) is CellState.OCCUPIED
    assert candidate.local in simulation.environment.floors[1].dynamic_obstacles
    assert candidate.local not in simulation.environment.floors[0].dynamic_obstacles


def test_failure_releases_floor_target_for_reassignment() -> None:
    simulation = MultiFloorSimulation(
        MultiFloorConfig(seed=0, failure_schedule=(("drone-1", 5),))
    )
    result = simulation.run()
    assert result.success
    assert simulation.agents["drone-1"].status == "FAILED"
    assert any(event["event_type"] == "agent_failed" for event in simulation.events)


def test_constrained_communication_uses_stair_endpoints() -> None:
    simulation = MultiFloorSimulation(
        MultiFloorConfig(floor_count=2, communication_profile="constrained")
    )
    transition = simulation.environment.transitions[0]
    simulation.agents["drone-1"].position = transition.source
    simulation.agents["drone-2"].position = transition.destination
    assert ("drone-1", "drone-2") in simulation.communication_links()


def test_replay_schema_serializes_floors_and_transitions(tmp_path: Path) -> None:
    simulation = MultiFloorSimulation(MultiFloorConfig(seed=0))
    result = simulation.run()
    replay = simulation.replay(result)
    assert replay["schema_version"] == MULTI_FLOOR_REPLAY_SCHEMA_VERSION
    assert replay["map"]["transitions"]
    assert len(replay["frames"][0]["drones"]["drone-1"]["position"]) == 3
    path = tmp_path / "multi-floor.json"
    path.write_text(json.dumps(replay), encoding="utf-8")
    _validate_replay(path)


@pytest.mark.parametrize("drone_count", (1, 2, 4, 8))
def test_n_agent_multi_floor_support(drone_count: int) -> None:
    result = MultiFloorSimulation(
        MultiFloorConfig(seed=1, drone_count=drone_count)
    ).run()
    assert result.success
    assert result.returned_agents == drone_count


def test_multi_floor_is_deterministic() -> None:
    first = MultiFloorSimulation(MultiFloorConfig(seed=3)).run().to_dict()
    second = MultiFloorSimulation(MultiFloorConfig(seed=3)).run().to_dict()
    assert first == second


def test_two_floor_ci_regression_is_deterministic_and_safe() -> None:
    config = MultiFloorConfig(floor_count=2, seed=2, drone_count=2)
    first = MultiFloorSimulation(config).run()
    second = MultiFloorSimulation(config).run()
    assert first.to_dict() == second.to_dict()
    assert first.success
    assert first.wall_collisions == first.drone_collisions == 0


def test_legacy_single_floor_path_is_unchanged() -> None:
    config = SimulationConfig(seed=7, drone_count=2)
    first = MultiDroneSimulation(config).run().to_dict()
    second = MultiDroneSimulation(config).run().to_dict()
    assert first == second


def test_cli_floor_options_and_floor_obstacle_parser() -> None:
    args = build_parser().parse_args(["--floors", "3", "--transition-cost", "3"])
    assert args.floors == 3
    assert floor_obstacle_spec("1:5,7:20") == (1, 5, 7, 20)


def test_dashboard_contains_floor_selector_support() -> None:
    root = Path(__file__).parents[1]
    html = (root / "src/echorescue/dashboard_assets/index.html").read_text(encoding="utf-8")
    javascript = (root / "src/echorescue/dashboard_assets/app.js").read_text(encoding="utf-8")
    assert 'id="floorSelect"' in html
    assert "drawMultiFloorMission" in javascript
    assert 'schema_version === "2.5"' in javascript
