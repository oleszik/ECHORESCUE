import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from echorescue.flight_integration import (
    _mission_checks,
    flight_smoke,
    load_flight_config,
)
from echorescue.sim_integration import Status, load_config


class MissionReportGateTests(unittest.TestCase):
    def test_pass_requires_exact_commands_accepted_and_later_state_evidence(self) -> None:
        commands = []
        for index, kind in enumerate(("extended_state_stream", "guided", "arm", "takeoff", "land"), start=1):
            commands.append({
                "kind": kind,
                "ack_result": 0,
                "ack_monotonic_ns": index * 100 + 1,
                "issued_monotonic_ns": index * 100,
                "telemetry_transition": f"observed {kind}",
                "telemetry_transition_monotonic_ns": index * 100 + 2,
            })
        mission = {
            "status": "PASS",
            "final_landed": True,
            "final_armed": False,
            "commands": commands,
        }
        observer = {"status": "PASS", "session_changed": False}
        self.assertTrue(all(item.status is Status.PASS for item in _mission_checks(mission, observer)))

    def test_ack_without_telemetry_transition_fails(self) -> None:
        mission = {
            "status": "PASS", "final_landed": True, "final_armed": False,
            "commands": [
                {"kind": kind, "ack_result": 0, "issued_monotonic_ns": 100, "telemetry_transition": None}
                for kind in ("extended_state_stream", "guided", "arm", "takeoff", "land")
            ],
        }
        checks = {item.name: item for item in _mission_checks(
            mission, {"status": "PASS", "session_changed": False}
        )}
        self.assertEqual(checks["mission.telemetry_transitions"].status, Status.FAIL)

    def test_transition_received_before_ack_fails(self) -> None:
        commands = [{
            "kind": kind,
            "ack_result": 0,
            "issued_monotonic_ns": 100,
            "ack_monotonic_ns": 102,
            "telemetry_transition": f"observed {kind}",
            "telemetry_transition_monotonic_ns": 101,
        } for kind in ("extended_state_stream", "guided", "arm", "takeoff", "land")]
        checks = {item.name: item for item in _mission_checks(
            {"status": "PASS", "final_landed": True, "final_armed": False, "commands": commands},
            {"status": "PASS", "session_changed": False},
        )}
        self.assertEqual(checks["mission.telemetry_transitions"].status, Status.FAIL)


class FlightScopeBoundaryTests(unittest.TestCase):
    def test_command_node_is_narrow_and_contains_no_ground_truth_or_navigation_api(self) -> None:
        source = (
            Path(__file__).parents[1]
            / "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_flight_mission.py"
        ).read_text(encoding="utf-8")
        self.assertIn("command_long_send", source)
        for prohibited in (
            "mission_item_send",
            "set_position_target",
            "rc_channels_override",
            "/world/",
            "gz.msgs",
            "Gazebo",
            "obstacle",
            "waypoint",
        ):
            self.assertNotIn(prohibited, source)

    def test_legacy_bridge_remains_receive_only(self) -> None:
        source = (
            Path(__file__).parents[1]
            / "ros2_ws/src/echorescue_ros/echorescue_ros/mavlink_telemetry_bridge.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("command_long_send", source)


class FlightSmokeLifecycleTests(unittest.TestCase):
    def test_unavailable_preflight_skips_without_starting_processes(self) -> None:
        config = load_flight_config()
        base = load_config()
        preflight = {
            "ready": False,
            "checks": [{
                "name": "ros_package.echorescue_ros",
                "status": Status.FAIL,
                "version": None,
                "detail": "not sourced",
            }],
        }
        with patch("echorescue.flight_integration.flight_diagnose", return_value=preflight):
            report = flight_smoke(config, base, 1.0, None)
        self.assertEqual(report["status"], Status.SKIP)
        self.assertEqual(report["cleanup"], [])

    def test_interruption_always_runs_owned_process_and_port_cleanup(self) -> None:
        config = load_flight_config()
        base = load_config()

        class InterruptingSupervisor:
            instance = None

            def __init__(self) -> None:
                self.cleaned = False
                InterruptingSupervisor.instance = self

            def start(self, name: str, command: list[str], cwd: Path, log: object) -> object:
                return object()

            def wait_until(self, name: str, process: object, predicate: object, timeout_s: float) -> None:
                raise KeyboardInterrupt

            def cleanup(self, grace_s: float = 5.0):
                self.cleaned = True
                return []

        with tempfile.TemporaryDirectory() as raw_dir:
            with (
                patch("echorescue.flight_integration.flight_diagnose", return_value={"ready": True, "checks": []}),
                patch("echorescue.flight_integration.ProcessSupervisor", InterruptingSupervisor),
                patch("echorescue.flight_integration._cleanup_port_checks", return_value=[]),
                patch.dict(os.environ, {"ARDUPILOT_HOME": raw_dir}),
            ):
                report = flight_smoke(config, base, 1.0, Path(raw_dir) / "report.json")
        self.assertEqual(report["status"], Status.FAIL)
        self.assertTrue(InterruptingSupervisor.instance.cleaned)


if __name__ == "__main__":
    unittest.main()
