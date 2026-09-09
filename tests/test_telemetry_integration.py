import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from echorescue.sim_integration import Check, Status, UnexpectedProcessExit, load_config
from echorescue.telemetry_integration import (
    _observer_checks,
    _wait_for_exit,
    load_telemetry_config,
    telemetry_smoke,
)


class ObserverReportTests(unittest.TestCase):
    def test_observer_checks_require_graph_advancing_samples_and_fresh_age(self) -> None:
        report = {
            "missing_nodes": [],
            "missing_topics": [],
            "vehicle_state_received": True,
            "global_position_received": True,
            "local_position_samples": 2,
            "local_source_time_boot_ms_first": 100,
            "local_source_time_boot_ms_last": 200,
            "telemetry_age_s": 1.0,
        }
        checks = _observer_checks(report, freshness_threshold_s=1.0)
        self.assertTrue(all(item.status is Status.PASS for item in checks))

    def test_stationary_duplicate_source_time_and_stale_age_fail(self) -> None:
        report = {
            "missing_nodes": [],
            "missing_topics": [],
            "vehicle_state_received": True,
            "global_position_received": True,
            "local_position_samples": 2,
            "local_source_time_boot_ms_first": 100,
            "local_source_time_boot_ms_last": 100,
            "telemetry_age_s": 1.0001,
        }
        checks = {item.name: item for item in _observer_checks(report, freshness_threshold_s=1.0)}
        self.assertEqual(checks["health.local_position"].status, Status.FAIL)
        self.assertEqual(checks["health.telemetry_age"].status, Status.FAIL)


class ProcessWaitTests(unittest.TestCase):
    class Process:
        def __init__(self, return_code: int | None) -> None:
            self.return_code = return_code

        def poll(self) -> int | None:
            return self.return_code

    def test_observer_success_and_failure_exit_are_distinguished(self) -> None:
        _wait_for_exit("observer", self.Process(0), 1.0)
        with self.assertRaises(UnexpectedProcessExit):
            _wait_for_exit("observer", self.Process(7), 1.0)


class TelemetrySmokeLifecycleTests(unittest.TestCase):
    def test_preflight_skip_has_exact_unavailable_checks_and_starts_nothing(self) -> None:
        config = load_telemetry_config()
        base = load_config()
        preflight = {
            "ready": False,
            "checks": [{
                "name": "ros_interface.mavlink_telemetry",
                "status": Status.FAIL,
                "version": None,
                "detail": "build and source the v0.14.1 ROS interfaces",
            }],
        }
        with patch("echorescue.telemetry_integration.telemetry_diagnose", return_value=preflight):
            report = telemetry_smoke(config, base, timeout_s=1.0, output_path=None)
        self.assertEqual(report["status"], Status.SKIP)
        self.assertEqual(report["detail"], "preflight unavailable: ros_interface.mavlink_telemetry")
        self.assertEqual(report["cleanup"], [])

    def test_interruption_runs_owned_process_and_port_cleanup(self) -> None:
        config = load_telemetry_config()
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

            def cleanup(self, grace_s: float = 5.0) -> list[Check]:
                self.cleaned = True
                return [Check("cleanup.gazebo", Status.PASS, None, "stopped")]

        with tempfile.TemporaryDirectory() as raw_dir:
            with (
                patch("echorescue.telemetry_integration.telemetry_diagnose", return_value={"ready": True, "checks": []}),
                patch("echorescue.telemetry_integration.ProcessSupervisor", InterruptingSupervisor),
                patch("echorescue.telemetry_integration._cleanup_port_checks", return_value=[Check("cleanup.port.sitl", Status.PASS, None, "available")]),
                patch.dict(os.environ, {"ARDUPILOT_HOME": raw_dir}),
            ):
                report = telemetry_smoke(config, base, timeout_s=1.0, output_path=Path(raw_dir) / "report.json")
        self.assertEqual(report["status"], Status.FAIL)
        self.assertTrue(InterruptingSupervisor.instance.cleaned)
        self.assertTrue(all(item["status"] == Status.PASS for item in report["cleanup"]))


if __name__ == "__main__":
    unittest.main()
