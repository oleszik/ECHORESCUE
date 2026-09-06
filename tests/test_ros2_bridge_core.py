import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from echorescue.bridge_artifacts import build_closed_loop_replay, write_json
from echorescue.bridge_contracts import (
    AgentStateReport,
    BridgeHealth,
    BridgeHeartbeat,
    CellObservation,
    CommandStatus,
    GRID_FRAME,
    GridCoordinateMapper,
    MovementCommand,
    SensorObservation,
)
from echorescue.closed_loop import (
    ClosedLoopConfig,
    ClosedLoopMission,
    ClosedLoopSimulatorBackend,
    MissionBridgeState,
    closed_loop_config,
)
from echorescue.closed_loop_demo import run_reference
from echorescue.config import SimulationConfig
from echorescue.dashboard import _validate_replay
from echorescue.models import CellState, Position
from echorescue.simulation import Simulation


SESSION = "test-session"
AGENT = "drone-1"


def small_config() -> SimulationConfig:
    return SimulationConfig(
        width=9,
        height=7,
        seed=7,
        obstacle_density=0.02,
        survivor_count=1,
        battery_capacity=300.0,
        drone_count=1,
    )


def synchronized_pair(now: int = 1_000_000_000) -> tuple[ClosedLoopMission, ClosedLoopSimulatorBackend]:
    config = small_config()
    backend = ClosedLoopSimulatorBackend(config)
    mission = ClosedLoopMission(SESSION, AGENT, closed_loop_config(config))
    assert backend.receive_mission_heartbeat(BridgeHeartbeat(SESSION, "mission", 1, 0.0, BridgeHealth.READY))
    assert mission.receive_backend_heartbeat(backend.heartbeat(), now)
    assert mission.receive_observation(backend.observation(), now)
    assert mission.receive_state(backend.state_report(), now)
    return mission, backend


class ContractTests(unittest.TestCase):
    def test_grid_cell_centres_and_inverse_mapping(self) -> None:
        mapper = GridCoordinateMapper(0.5, 10.0, -2.0, 3.0)
        self.assertEqual(mapper.grid_to_metric(Position(2, 3)), (11.25, -0.25, 3.0))
        self.assertEqual(mapper.metric_to_grid(11.49, -0.01), Position(2, 3))

    def test_contract_validation_rejects_untyped_or_invalid_data(self) -> None:
        with self.assertRaises(ValueError):
            CellObservation(Position(1, 1), CellState.UNKNOWN, 0.9)
        with self.assertRaises(ValueError):
            CellObservation(Position(1, 1), CellState.FREE, 1.1)
        with self.assertRaises(ValueError):
            MovementCommand(SESSION, AGENT, "bad command", 0, 0.0, 1.0, Position(1, 1), Position(2, 1))

    def test_command_requires_one_cardinal_step_and_positive_validity(self) -> None:
        with self.assertRaises(ValueError):
            MovementCommand(SESSION, AGENT, "cmd-1", 0, 0.0, 1.0, Position(1, 1), Position(2, 2))
        with self.assertRaises(ValueError):
            MovementCommand(SESSION, AGENT, "cmd-1", 0, 1.0, 1.0, Position(1, 1), Position(2, 1))


class MissionBoundaryTests(unittest.TestCase):
    def test_ground_truth_is_confined_to_backend(self) -> None:
        mission, backend = synchronized_pair()
        self.assertFalse(hasattr(mission, "world"))
        self.assertTrue(hasattr(backend, "world"))

    def test_duplicate_and_out_of_order_inputs_are_ignored(self) -> None:
        mission, backend = synchronized_pair()
        observation = backend.observation()
        self.assertFalse(mission.receive_observation(observation, 1_000_000_001))
        stale = SensorObservation(SESSION, AGENT, "different-old-id", 0, 0.0, GRID_FRAME, backend.position, observation.cells)
        self.assertFalse(mission.receive_observation(stale, 1_000_000_002))
        self.assertFalse(mission.receive_state(backend.state_report(), 1_000_000_003))

    def test_previous_session_messages_never_change_new_mission(self) -> None:
        mission, backend = synchronized_pair()
        old = backend.observation()
        foreign = SensorObservation("previous-session", AGENT, "old-obs", 99, 99.0, GRID_FRAME, Position(7, 5), old.cells)
        self.assertFalse(mission.receive_observation(foreign))
        self.assertEqual(mission.agent_state.position, Position(1, 1))

    def test_success_is_not_assumed_before_result_and_state_feedback(self) -> None:
        mission, backend = synchronized_pair()
        command = mission.tick(1_001_000_000)
        self.assertIsNotNone(command)
        assert command is not None
        result = backend.execute(command)
        mission.receive_result(result)
        self.assertEqual(mission.state, MissionBridgeState.WAITING_FOR_COMMAND)
        mission.receive_observation(backend.observation(), 1_002_000_000)
        self.assertEqual(mission.state, MissionBridgeState.WAITING_FOR_COMMAND)
        mission.receive_state(backend.state_report(), 1_002_000_000)
        self.assertEqual(mission.state, MissionBridgeState.ACTIVE)

    def test_command_timeout_enters_controlled_stop_and_blocks_new_commands(self) -> None:
        mission, _ = synchronized_pair()
        self.assertIsNotNone(mission.tick(1_001_000_000))
        self.assertIsNone(mission.tick(4_000_000_000))
        self.assertEqual(mission.state, MissionBridgeState.COMMUNICATION_ERROR)
        self.assertIsNone(mission.tick(4_000_000_001))
        self.assertEqual(mission.state, MissionBridgeState.CONTROLLED_STOP)
        self.assertIsNone(mission.tick(5_000_000_000))

    def test_resume_requires_fresh_synchronized_state_and_explicit_authorization(self) -> None:
        mission, _ = synchronized_pair()
        mission.request_stop("test")
        mission.tick(1_000_000_001)
        self.assertTrue(mission.authorize_resume(1_000_000_002))
        self.assertEqual(mission.state, MissionBridgeState.ACTIVE)


class BackendLifecycleTests(unittest.TestCase):
    def test_duplicate_command_has_exactly_once_execution(self) -> None:
        mission, backend = synchronized_pair()
        command = mission.tick(1_001_000_000)
        assert command is not None
        first = backend.execute(command)
        second = backend.execute(command)
        self.assertEqual(first, second)
        self.assertEqual(backend.state_sequence, 1)

    def test_state_precondition_rejects_conflicting_command(self) -> None:
        _, backend = synchronized_pair()
        command = MovementCommand(SESSION, AGENT, "cmd-conflict", 99, 0.0, 2.0, Position(1, 1), Position(2, 1))
        result = backend.execute(command)
        self.assertEqual(result.status, CommandStatus.REJECTED)
        self.assertEqual(backend.position, Position(1, 1))

    def test_process_loss_expires_backend_lease(self) -> None:
        clock = [0]
        config = small_config()
        backend = ClosedLoopSimulatorBackend(config, command_lease_timeout_s=1.0, clock_ns=lambda: clock[0])
        backend.receive_mission_heartbeat(BridgeHeartbeat(SESSION, "mission", 1, 0.0, BridgeHealth.READY))
        clock[0] = 1_000_000_001
        self.assertFalse(backend.watchdog())
        self.assertTrue(backend.stopped)
        self.assertEqual(backend.error, "mission_heartbeat_timeout")

    def test_restart_requires_expired_lease_and_separates_sessions(self) -> None:
        clock = [0]
        backend = ClosedLoopSimulatorBackend(small_config(), command_lease_timeout_s=1.0, clock_ns=lambda: clock[0])
        self.assertTrue(backend.receive_mission_heartbeat(BridgeHeartbeat("session-one", "mission", 1, 0.0, BridgeHealth.READY)))
        self.assertFalse(backend.receive_mission_heartbeat(BridgeHeartbeat("session-two", "mission", 1, 0.0, BridgeHealth.READY)))
        clock[0] = 1_000_000_001
        self.assertTrue(backend.receive_mission_heartbeat(BridgeHeartbeat("session-two", "mission", 2, 0.0, BridgeHealth.READY)))
        old = MovementCommand("session-one", AGENT, "old-command", 0, 0.0, 2.0, Position(1, 1), Position(2, 1))
        self.assertEqual(backend.execute(old).status, CommandStatus.REJECTED)
        self.assertEqual(backend.state_sequence, 0)


class ClosedLoopIntegrationTests(unittest.TestCase):
    def test_reference_transport_completes_real_exploration_and_return(self) -> None:
        mission, backend = run_reference(small_config())
        report = mission.report()
        self.assertEqual(mission.state, MissionBridgeState.COMPLETED)
        self.assertTrue(report["mission_success"])
        self.assertGreater(report["commands_issued"], 4)
        self.assertEqual(backend.shield_interventions, 0)

    def test_direct_and_bridge_runs_are_consistent_on_mission_outcomes(self) -> None:
        config = small_config()
        direct = Simulation(config).run()
        mission, _ = run_reference(config)
        bridge = mission.report()
        self.assertEqual(bridge["mission_success"], direct.mission_success)
        self.assertEqual(bridge["returned_to_base"], direct.returned_to_base)
        self.assertEqual(bridge["survivors_confirmed"], direct.survivors_confirmed)

    def test_bridge_replay_is_accepted_by_existing_dashboard(self) -> None:
        mission, _ = run_reference(small_config())
        with TemporaryDirectory() as directory:
            path = write_json(build_closed_loop_replay(mission), Path(directory) / "replay.json")
            _validate_replay(path)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertTrue(payload["frames"])
            self.assertIn("bridge", payload["frames"][-1])

    def test_legacy_single_agent_operation_remains_unchanged(self) -> None:
        first = Simulation(small_config()).run().to_dict()
        second = Simulation(small_config()).run().to_dict()
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
