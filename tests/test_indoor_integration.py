import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from echorescue.indoor_integration import _checks, _mission_command, indoor_smoke, load_configured_stack
from echorescue.sim_integration import Status


CONFIG = Path(__file__).parents[1] / "config/indoor-reference-v0.14.5.json"


def reports():
    raw, _ = load_configured_stack(CONFIG)
    expected = [item["target_id"] for item in raw["mission"]["targets"]]
    commands = [{"ack_result": 0, "ack_monotonic_ns": n, "telemetry_transition_monotonic_ns": n + 1} for n in range(5)]
    targets = [{"target_id": target} for target in expected]
    mission = {"status": "PASS", "commands": commands, "targets": targets, "final_landed": True, "final_armed": False,
               "final_position_enu": {"east_m": 0.0, "north_m": 0.0, "up_m": 0.0}}
    mission["launch_enu"] = {"east_m": 0.0, "north_m": 0.0, "up_m": 0.0}
    observer = {"target_ids": expected, "settled_event_ids": expected}
    gazebo = {"status": "PASS", "prohibited_contact_detected": False, "contact_count": 0,
              "minimum_obstacle_surface_clearance_m": 0.3, "minimum_doorway_boundary_surface_clearance_m": 0.3,
              "trajectory": [{"east_m": -4, "north_m": 0, "up_m": .2}, {"east_m": -4, "north_m": 0, "up_m": .2}]}
    return raw, mission, observer, gazebo


class IndoorIntegrationGateTests(unittest.TestCase):
    def test_passing_evidence_satisfies_all_gates(self) -> None:
        raw, mission, observer, gazebo = reports()
        self.assertTrue(all(item.status is Status.PASS for item in _checks(raw, mission, observer, gazebo, 0)))

    def test_collision_fails_acceptance(self) -> None:
        raw, mission, observer, gazebo = reports()
        gazebo["status"] = "FAIL"
        gazebo["prohibited_contact_detected"] = True
        checks = {item.name: item for item in _checks(raw, mission, observer, gazebo, 1)}
        self.assertEqual(checks["gazebo.collision_free_route"].status, Status.FAIL)

    def test_evidence_disagreement_fails_acceptance(self) -> None:
        raw, mission, observer, gazebo = reports()
        gazebo["trajectory"][-1]["east_m"] = 2
        checks = {item.name: item for item in _checks(raw, mission, observer, gazebo, 0)}
        self.assertEqual(checks["evidence.telemetry_gazebo_agreement"].status, Status.FAIL)

    def test_mission_command_uses_only_existing_waypoint_node(self) -> None:
        raw, _ = load_configured_stack(CONFIG)
        command = _mission_command(raw, Path("report.json"))
        self.assertEqual(command[1:3], ["-m", "echorescue_ros.mavlink_waypoint_mission"])
        self.assertTrue(any("targets_json:=" in item for item in command))

    def test_gui_configuration_is_forwarded_to_owned_gazebo_command(self) -> None:
        gui_config = CONFIG.parent / "portfolio-gui.config"
        raw = {
            "base_stack_config": "unused.json",
            "runner": {"node_name": "mission", "observer_node_name": "observer"},
            "logging": {"failure_log_max_bytes": 1024},
            "dependencies": {},
        }
        supervisor = MagicMock()
        supervisor.processes = []
        supervisor.cleanup.return_value = []
        supervisor.start.side_effect = RuntimeError("stop after capturing Gazebo command")
        simulated_config = SimpleNamespace(world_sdf="unused.sdf", world_name="test", world_version="test")

        with (
            patch("echorescue.indoor_integration.load_configured_stack", return_value=(raw, simulated_config)),
            patch("echorescue.indoor_integration.indoor_diagnose", return_value={"ready": True}),
            patch("echorescue.indoor_integration.load_config", return_value={}),
            patch("echorescue.indoor_integration.ProcessSupervisor", return_value=supervisor),
            patch("echorescue.indoor_integration._ros_nodes_absent", return_value=True),
            patch("echorescue.indoor_integration._cleanup_port_checks", return_value=[]),
            patch("echorescue.indoor_integration._retain_failure_logs", return_value=[]),
        ):
            indoor_smoke(CONFIG, None, 1.0, True, "owned", gui_config)

        gazebo_command = supervisor.start.call_args.args[1]
        gui_option = gazebo_command.index("--gui-config")
        self.assertEqual(gazebo_command[gui_option + 1], str(gui_config.resolve()))

    def test_gui_configuration_requires_graphical_owned_stack(self) -> None:
        gui_config = CONFIG.parent / "portfolio-gui.config"
        with self.assertRaisesRegex(ValueError, "graphical mode"):
            indoor_smoke(CONFIG, None, 1.0, False, "owned", gui_config)
        with self.assertRaisesRegex(ValueError, "owned stack"):
            indoor_smoke(CONFIG, None, 1.0, True, "attached", gui_config)
