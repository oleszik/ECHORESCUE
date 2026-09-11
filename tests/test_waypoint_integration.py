import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from echorescue.sim_integration import Status, load_config
from echorescue.waypoint_integration import (
    _mission_checks,
    load_waypoint_config,
    validate_waypoint_config,
    waypoint_smoke,
)


def passing_reports() -> tuple[dict[str, object], dict[str, object]]:
    commands = []
    for index, kind in enumerate(("extended_state_stream", "guided", "arm", "takeoff", "land"), 1):
        commands.append({
            "kind": kind, "ack_result": 0, "ack_monotonic_ns": index * 100 + 1,
            "telemetry_transition_monotonic_ns": index * 100 + 2,
        })
    targets = []
    for index, target_id in enumerate(("waypoint-a", "waypoint-b", "return-launch"), 1):
        targets.append({
            "target_id": target_id, "outcome": "settled",
            "transmit_monotonic_ns": index * 1_000,
            "first_post_command_monotonic_ns": index * 1_000 + 1,
            "transmit_source_time_boot_ms": index * 100,
            "first_post_command_source_time_boot_ms": index * 100 + 1,
            "horizontal_error_m": 0.1, "vertical_error_m": 0.1,
            "settling_duration_s": 2.0,
        })
    mission: dict[str, object] = {
        "status": "PASS", "session_id": "s", "commands": commands, "targets": targets,
        "final_landed": True, "final_armed": False, "final_distance_from_launch_m": 0.2,
    }
    observer: dict[str, object] = {
        "status": "PASS", "session_id": "s", "session_changed": False,
        "target_ids": ["waypoint-a", "waypoint-b", "return-launch"],
        "settled_event_ids": ["waypoint-a", "waypoint-b", "return-launch"],
    }
    return mission, observer


class WaypointReportGateTests(unittest.TestCase):
    def test_repository_configuration_validates(self) -> None:
        validate_waypoint_config(load_waypoint_config())

    def test_mission_transport_uses_the_validated_harness_python(self) -> None:
        command = load_waypoint_config()["runner"]["command"]
        self.assertEqual(
            command[:3],
            ["{transport_python}", "-m", "echorescue_ros.mavlink_waypoint_mission"],
        )

    def test_configuration_rejects_non_loopback_command_endpoint(self) -> None:
        config = load_waypoint_config()
        config["runner"]["endpoint"] = "tcp:192.0.2.1:5760"
        with self.assertRaisesRegex(ValueError, "loopback TCP SITL"):
            validate_waypoint_config(config)

    def test_acceptance_requires_ack_transitions_and_post_setpoint_evidence(self) -> None:
        mission, observer = passing_reports()
        checks = _mission_checks(mission, observer, load_waypoint_config())
        self.assertTrue(all(item.status is Status.PASS for item in checks))

    def test_precommand_completion_evidence_fails(self) -> None:
        mission, observer = passing_reports()
        targets = mission["targets"]
        assert isinstance(targets, list)
        targets[0]["first_post_command_monotonic_ns"] = targets[0]["transmit_monotonic_ns"]
        checks = {item.name: item for item in _mission_checks(mission, observer, load_waypoint_config())}
        self.assertEqual(checks["mission.target_post_command_evidence"].status, Status.FAIL)

    def test_observer_disagreement_fails(self) -> None:
        mission, observer = passing_reports()
        observer["settled_event_ids"] = ["waypoint-a"]
        checks = {item.name: item for item in _mission_checks(mission, observer, load_waypoint_config())}
        self.assertEqual(checks["mission.independent_observer_agreement"].status, Status.FAIL)


class WaypointScopeTests(unittest.TestCase):
    def test_navigation_node_excludes_ground_truth_rc_and_general_mission_protocol(self) -> None:
        source = (Path(__file__).parents[1] / "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_waypoint_mission.py").read_text(encoding="utf-8")
        self.assertIn("set_position_target_local_ned_send", source)
        for prohibited in ("mission_item_send", "rc_channels_override", "/world/", "gz.msgs", "Gazebo"):
            self.assertNotIn(prohibited, source)

    def test_legacy_bridge_is_still_command_free(self) -> None:
        source = (Path(__file__).parents[1] / "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_telemetry_bridge.py").read_text(encoding="utf-8")
        self.assertNotIn("command_long_send", source)
        self.assertNotIn("set_position_target_local_ned_send", source)


class WaypointLifecycleTests(unittest.TestCase):
    def test_unavailable_owned_preflight_starts_nothing(self) -> None:
        config, base = load_waypoint_config(), load_config()
        preflight = {"ready": False, "checks": [{"name": "dependency", "status": Status.FAIL}]}
        with patch("echorescue.waypoint_integration.waypoint_diagnose", return_value=preflight):
            report = waypoint_smoke(config, base, 1.0, None, stack_mode="owned")
        self.assertEqual(report["status"], Status.SKIP)
        self.assertEqual(report["cleanup"], [])

    def test_owned_interruption_always_cleans_processes_and_ports(self) -> None:
        config, base = load_waypoint_config(), load_config()

        class InterruptingSupervisor:
            instance = None

            def __init__(self) -> None:
                self.cleaned = False
                self.processes = []
                InterruptingSupervisor.instance = self

            def start(self, name: str, command: list[str], cwd: Path, log: object) -> object:
                process = type("Process", (), {"poll": lambda self: None})()
                self.processes.append((name, process))
                return process

            def wait_until(self, name: str, process: object, predicate: object, timeout_s: float) -> None:
                raise KeyboardInterrupt

            def cleanup(self):
                self.cleaned = True
                return []

        with tempfile.TemporaryDirectory() as raw_dir:
            with (
                patch("echorescue.waypoint_integration.waypoint_diagnose", return_value={"ready": True, "checks": []}),
                patch("echorescue.waypoint_integration.ProcessSupervisor", InterruptingSupervisor),
                patch("echorescue.waypoint_integration._cleanup_port_checks", return_value=[]),
                patch.dict(os.environ, {"ARDUPILOT_HOME": raw_dir}),
            ):
                report = waypoint_smoke(config, base, 1.0, Path(raw_dir) / "report.json", stack_mode="owned")
        self.assertEqual(report["status"], Status.FAIL)
        self.assertTrue(InterruptingSupervisor.instance.cleaned)

    def test_attached_mode_does_not_apply_external_port_cleanup(self) -> None:
        config, base = load_waypoint_config(), load_config()
        with patch("echorescue.waypoint_integration._attached_preflight", return_value={"ready": False, "checks": [{"name": "x", "status": Status.FAIL}]}), patch("echorescue.waypoint_integration._cleanup_port_checks") as cleanup:
            waypoint_smoke(config, base, 1.0, None, stack_mode="attached")
        cleanup.assert_not_called()

    def test_startup_timeout_runs_owned_cleanup(self) -> None:
        from echorescue.sim_integration import StartupTimeout

        config, base = load_waypoint_config(), load_config()

        class TimingOutSupervisor:
            instance = None

            def __init__(self) -> None:
                self.cleaned = False
                self.processes = []
                TimingOutSupervisor.instance = self

            def start(self, name: str, command: list[str], cwd: Path, log: object) -> object:
                process = type("Process", (), {"poll": lambda self: 0})()
                self.processes.append((name, process))
                return process

            def wait_until(self, name: str, process: object, predicate: object, timeout_s: float) -> None:
                raise StartupTimeout("deterministic startup timeout")

            def cleanup(self):
                self.cleaned = True
                return []

        with tempfile.TemporaryDirectory() as raw_dir:
            with (
                patch("echorescue.waypoint_integration.waypoint_diagnose", return_value={"ready": True, "checks": []}),
                patch("echorescue.waypoint_integration.ProcessSupervisor", TimingOutSupervisor),
                patch("echorescue.waypoint_integration._cleanup_port_checks", return_value=[]),
                patch.dict(os.environ, {"ARDUPILOT_HOME": raw_dir}),
            ):
                report = waypoint_smoke(config, base, 1.0, None, stack_mode="owned")
        self.assertEqual(report["status"], Status.FAIL)
        self.assertTrue(TimingOutSupervisor.instance.cleaned)


if __name__ == "__main__":
    unittest.main()
