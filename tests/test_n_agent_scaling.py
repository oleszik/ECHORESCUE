import json
import subprocess
import unittest
from pathlib import Path

from echorescue.config import SimulationConfig
from echorescue.coordination import assign_frontiers, resolve_movements
from echorescue.events import EventType
from echorescue.mapping import OccupancyMap
from echorescue.models import CellState, DroneStatus, Position
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import generate_replay, replay_json_bytes
from echorescue.scaling_benchmark import run_benchmark


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
APP_PATH = REPOSITORY_ROOT / "src" / "echorescue" / "dashboard_assets" / "app.js"
FLEET_SIZES = (1, 2, 4, 8)


def known_free_map() -> OccupancyMap:
    occupancy = OccupancyMap(7, 7)
    occupancy.update(
        {
            Position(x, y): (
                CellState.OCCUPIED
                if x in {0, 6} or y in {0, 6}
                else CellState.FREE
            )
            for y in range(7)
            for x in range(7)
        }
    )
    return occupancy


class FleetInitializationTests(unittest.TestCase):
    def test_supported_fleet_sizes_have_stable_unique_ids_and_starts(self) -> None:
        for fleet_size in FLEET_SIZES:
            with self.subTest(fleet_size=fleet_size):
                first = MultiDroneSimulation(
                    SimulationConfig(seed=7, drone_count=fleet_size)
                )
                second = MultiDroneSimulation(
                    SimulationConfig(seed=7, drone_count=fleet_size)
                )
                expected_ids = {
                    f"drone-{index}" for index in range(1, fleet_size + 1)
                }
                self.assertEqual(set(first.runtimes), expected_ids)
                self.assertEqual(
                    [runtime.drone.position for runtime in first.runtimes.values()],
                    [runtime.drone.position for runtime in second.runtimes.values()],
                )
                starts = [
                    runtime.drone.position for runtime in first.runtimes.values()
                ]
                self.assertEqual(len(starts), len(set(starts)))

    def test_shared_base_is_an_explicit_safe_colocation_mode(self) -> None:
        starts = tuple((1, 1) for _ in range(8))
        simulation = MultiDroneSimulation(
            SimulationConfig(
                seed=7,
                drone_count=8,
                drone_start_positions=starts,
            )
        )
        self.assertEqual(
            {runtime.drone.position for runtime in simulation.runtimes.values()},
            {simulation.world.base},
        )


class FleetCoordinationTests(unittest.TestCase):
    def test_excess_agents_do_not_duplicate_or_require_assignments(self) -> None:
        positions = {
            f"drone-{index}": Position(index, 1) for index in range(1, 5)
        }
        assignments = assign_frontiers(
            positions,
            (Position(1, 5), Position(5, 5)),
            known_free_map(),
            {drone_id: None for drone_id in positions},
        )
        self.assertLessEqual(len(assignments), 2)
        self.assertEqual(
            len({assignment.target for assignment in assignments.values()}),
            len(assignments),
        )

    def test_three_agent_convergence_has_stable_single_winner(self) -> None:
        current = {
            "drone-1": Position(2, 1),
            "drone-2": Position(1, 2),
            "drone-3": Position(3, 2),
        }
        destination = Position(2, 2)
        resolved, conflicts = resolve_movements(
            current,
            {drone_id: destination for drone_id in current},
            base=Position(1, 1),
        )
        self.assertEqual(resolved["drone-1"], destination)
        self.assertEqual(resolved["drone-2"], current["drone-2"])
        self.assertEqual(resolved["drone-3"], current["drone-3"])
        self.assertEqual(conflicts[-1].drone_ids, ("drone-1", "drone-2", "drone-3"))

    def test_three_agent_cycle_is_blocked_without_iteration_bias(self) -> None:
        current = {
            "drone-1": Position(1, 1),
            "drone-2": Position(2, 1),
            "drone-3": Position(2, 2),
        }
        intended = {
            "drone-1": current["drone-2"],
            "drone-2": current["drone-3"],
            "drone-3": current["drone-1"],
        }
        resolved, _ = resolve_movements(current, intended, base=Position(4, 4))
        self.assertEqual(resolved, current)


class FleetIntegrationTests(unittest.TestCase):
    def test_all_fleet_sizes_are_deterministic_safe_and_complete(self) -> None:
        for fleet_size in FLEET_SIZES:
            with self.subTest(fleet_size=fleet_size):
                config = SimulationConfig(seed=0, drone_count=fleet_size)
                first = MultiDroneSimulation(config).run()
                second = MultiDroneSimulation(config).run()
                self.assertEqual(first, second)
                self.assertTrue(first.mission_success)
                self.assertEqual(first.drones_returned, fleet_size)
                self.assertEqual(first.drone_drone_collisions, 0)
                self.assertTrue(
                    all(
                        status is DroneStatus.LANDED
                        for status in first.drone_status_by_drone.values()
                    )
                )

    def test_four_agent_failure_is_reassigned_once(self) -> None:
        result = MultiDroneSimulation(
            SimulationConfig(
                seed=7,
                drone_count=4,
                failure_schedule=(("drone-1", 4),),
            )
        ).run()
        reassignments = [
            event
            for event in result.mission_events
            if event.event_type is EventType.FAILURE_TASK_REASSIGNED
        ]
        self.assertTrue(result.mission_success)
        self.assertEqual(result.drones_failed, 1)
        self.assertEqual(result.drones_returned, 3)
        self.assertEqual(len(reassignments), 1)
        self.assertNotEqual(reassignments[0].drone_id, "drone-1")

    def test_four_and_eight_agent_replays_are_deterministic_collections(self) -> None:
        for fleet_size in (4, 8):
            with self.subTest(fleet_size=fleet_size):
                config = SimulationConfig(seed=0, drone_count=fleet_size)
                first = generate_replay(config)
                second = generate_replay(config)
                self.assertEqual(replay_json_bytes(first), replay_json_bytes(second))
                expected = {
                    f"drone-{index}" for index in range(1, fleet_size + 1)
                }
                self.assertTrue(
                    all(set(frame["drones"]) == expected for frame in first["frames"])
                )

    def test_dashboard_validator_accepts_eight_agents(self) -> None:
        frame = {
            "step": 0,
            "drones": {f"drone-{index}": {} for index in range(1, 9)},
        }
        replay = {"schema_version": "2.0", "frames": [frame]}
        script = (
            f"const app=require({json.dumps(str(APP_PATH))});"
            "let s='';process.stdin.on('data',c=>s+=c);"
            "process.stdin.on('end',()=>{app.validateReplay(JSON.parse(s));"
            "process.stdout.write('ok')});"
        )
        completed = subprocess.run(
            ["node", "-e", script],
            input=json.dumps(replay),
            text=True,
            capture_output=True,
            check=True,
        )
        self.assertEqual(completed.stdout, "ok")

    def test_small_scaling_benchmark_is_deterministic_and_complete(self) -> None:
        first = run_benchmark(seed_count=2)
        second = run_benchmark(seed_count=2)
        self.assertEqual(first, second)
        self.assertEqual(first["determinism_check"], "passed")
        self.assertEqual(set(first["aggregates"]), {"1", "2", "4", "8"})
        self.assertTrue(all(first["acceptance"].values()))
        script = (
            f"const app=require({json.dumps(str(APP_PATH))});"
            "let s='';process.stdin.on('data',c=>s+=c);"
            "process.stdin.on('end',()=>process.stdout.write("
            "JSON.stringify(app.safeBenchmarkView(JSON.parse(s)))));"
        )
        completed = subprocess.run(
            ["node", "-e", script],
            input=json.dumps(first),
            text=True,
            capture_output=True,
            check=True,
        )
        view = json.loads(completed.stdout)
        self.assertEqual(view["status"], "ready")
        self.assertEqual(view["format"], "n_agent_scaling")


if __name__ == "__main__":
    unittest.main()
