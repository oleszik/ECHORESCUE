import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from echorescue.sim_integration import (
    Check,
    ProcessSupervisor,
    StartupTimeout,
    Status,
    UnexpectedProcessExit,
    _mavlink_samples,
    _retain_failure_logs,
    diagnose,
    expected_lines,
    load_config,
    parse_version,
    serialize_report,
    smoke,
    telemetry_progressed,
)


class DiagnosticTests(unittest.TestCase):
    def test_version_parser(self) -> None:
        self.assertEqual(
            parse_version("Gazebo Sim, version 8.9.0", r"version\s+([0-9.]+)"),
            "8.9.0",
        )
        self.assertIsNone(parse_version("no version", r"version\s+([0-9.]+)"))

    def test_dependency_and_version_detection_accepts_pinned_baseline(self) -> None:
        config = load_config()
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root)
            (root / "ros2_ws" / "src" / "echorescue_ros").mkdir(parents=True)
            (root / "ros2_ws" / "src" / "echorescue_ros" / "package.xml").write_text("<package/>")
            ardupilot = root / "ardupilot"
            (ardupilot / "Tools" / "autotest").mkdir(parents=True)
            (ardupilot / "Tools" / "autotest" / "sim_vehicle.py").write_text("")
            (ardupilot / "build" / "sitl" / "bin").mkdir(parents=True)
            (ardupilot / "build" / "sitl" / "bin" / "arducopter").write_text("")
            plugin = root / "ardupilot_gazebo"
            (plugin / "build").mkdir(parents=True)
            environment = {
                "ROS_DISTRO": "jazzy",
                "GZ_VERSION": "harmonic",
                "ARDUPILOT_HOME": str(ardupilot),
                "ARDUPILOT_GAZEBO_HOME": str(plugin),
                "GZ_SIM_SYSTEM_PLUGIN_PATH": str(plugin / "build"),
                "GZ_SIM_RESOURCE_PATH": os.pathsep.join((str(plugin / "models"), str(plugin / "worlds"))),
            }

            def command_result(command: list[str]) -> tuple[int, str]:
                if command[-2:] == ["rev-parse", "HEAD"]:
                    if str(plugin) in command:
                        return 0, config["baseline"]["ardupilot_gazebo"]["ref"]
                    return 0, config["baseline"]["ardupilot"]["commit"]
                if "gz" in command[0]:
                    return 0, "Gazebo Sim, version 8.9.0"
                if command[-1:] == ["--help"]:
                    return 0, "-N, --no-rebuild don't rebuild before starting ardupilot"
                return 0, "available"

            with (
                patch.dict(os.environ, environment, clear=True),
                patch("echorescue.sim_integration.platform.system", return_value="Linux"),
                patch("echorescue.sim_integration.platform.machine", return_value="x86_64"),
                patch("echorescue.sim_integration._ubuntu_version", return_value="24.04"),
                patch("echorescue.sim_integration.shutil.which", side_effect=lambda name: f"/usr/bin/{name}"),
                patch("echorescue.sim_integration._run_version", side_effect=command_result),
                patch("echorescue.sim_integration._package_version", return_value="2.4.49"),
                patch("echorescue.sim_integration._port_check", side_effect=lambda name, host, port, protocol: Check(name, Status.PASS, None, "available")),
                patch("echorescue.sim_integration._rendering_check", return_value=Check("rendering", Status.PASS, "NVIDIA", "accelerated")),
            ):
                report = diagnose(config, root)
        self.assertTrue(report["ready"])
        self.assertTrue(all(item["status"] == "PASS" for item in report["checks"]))

    def test_diagnostic_rejects_missing_prebuilt_sitl_binary(self) -> None:
        config = load_config()
        with tempfile.TemporaryDirectory() as raw_root:
            ardupilot = Path(raw_root) / "ardupilot"
            (ardupilot / "Tools" / "autotest").mkdir(parents=True)
            (ardupilot / "Tools" / "autotest" / "sim_vehicle.py").write_text("")
            with (
                patch.dict(os.environ, {"ARDUPILOT_HOME": str(ardupilot)}, clear=True),
                patch("echorescue.sim_integration.shutil.which", return_value=None),
                patch("echorescue.sim_integration._run_version", return_value=(0, "--no-rebuild")),
                patch("echorescue.sim_integration._ubuntu_version", return_value="24.04"),
                patch("echorescue.sim_integration._rendering_check", return_value=Check("rendering", Status.SKIP, None, "headless")),
            ):
                report = diagnose(config, Path(raw_root))
        binary_check = next(item for item in report["checks"] if item["name"] == "ardupilot_sitl.binary")
        self.assertEqual(binary_check["status"], Status.FAIL)
        self.assertIn("build/sitl/bin/arducopter", binary_check["detail"])

    def test_pinned_sitl_command_disables_rebuild(self) -> None:
        command = load_config()["integration"]["sitl_command"]
        self.assertIn("--no-rebuild", command)

    def test_serialization_is_deterministic(self) -> None:
        left = {"z": 1, "a": [{"status": Status.PASS, "name": "x"}]}
        right = {"a": [{"name": "x", "status": Status.PASS}], "z": 1}
        self.assertEqual(serialize_report(left), serialize_report(right))
        self.assertEqual(json.loads(serialize_report(left)), json.loads(serialize_report(right)))


class HealthParsingTests(unittest.TestCase):
    def test_expected_ros_entities(self) -> None:
        ok, missing = expected_lines("/ardupilot_dds\n/rosout\n", ["/rosout", "/ardupilot_dds"])
        self.assertTrue(ok)
        self.assertEqual(missing, [])
        ok, missing = expected_lines("/rosout\n", ["/ardupilot_dds"])
        self.assertFalse(ok)
        self.assertEqual(missing, ["/ardupilot_dds"])

    def test_telemetry_requires_an_advancing_boot_timestamp(self) -> None:
        self.assertFalse(telemetry_progressed([
            {"time_boot_ms": 1, "x": 0.0},
            {"time_boot_ms": 1, "x": 3.0},
        ]))
        self.assertFalse(telemetry_progressed([{"x": 0.0}, {"x": 3.0}]))
        self.assertTrue(telemetry_progressed([
            {"time_boot_ms": 1, "x": 0.0},
            {"time_boot_ms": 2, "x": 0.0},
        ]))

    def test_position_stream_is_requested_after_heartbeat_at_configured_rate(self) -> None:
        connection = MagicMock()
        connection.target_system = 1
        connection.target_component = 1
        connection.wait_heartbeat.return_value = object()
        connection.recv_match.side_effect = [
            MagicMock(to_dict=lambda: {"time_boot_ms": 100, "x": 0.0}),
            MagicMock(to_dict=lambda: {"time_boot_ms": 200, "x": 0.0}),
        ]
        mavutil = SimpleNamespace(
            mavlink_connection=MagicMock(return_value=connection),
            mavlink=SimpleNamespace(MAV_DATA_STREAM_POSITION=6),
        )
        with patch.dict(sys.modules, {"pymavlink": SimpleNamespace(mavutil=mavutil)}):
            samples = _mavlink_samples("tcp:127.0.0.1:5760", 2, 1.0, 10)
        self.assertEqual(len(samples), 2)
        connection.wait_heartbeat.assert_called_once()
        connection.mav.request_data_stream_send.assert_called_once()
        request = connection.mav.request_data_stream_send.call_args.args
        self.assertEqual(request[:2], (1, 1))
        self.assertEqual(request[3:], (10, 1))
        connection.close.assert_called_once()

    def test_missing_position_telemetry_times_out_and_closes_connection(self) -> None:
        connection = MagicMock()
        connection.target_system = 1
        connection.target_component = 1
        connection.wait_heartbeat.return_value = object()
        connection.recv_match.return_value = None
        mavutil = SimpleNamespace(
            mavlink_connection=MagicMock(return_value=connection),
            mavlink=SimpleNamespace(MAV_DATA_STREAM_POSITION=6),
        )
        with (
            patch.dict(sys.modules, {"pymavlink": SimpleNamespace(mavutil=mavutil)}),
            patch("echorescue.sim_integration.time.monotonic", side_effect=[0.0, 0.0, 0.0, 2.0]),
        ):
            samples = _mavlink_samples("tcp:127.0.0.1:5760", 2, 1.0, 10)
        self.assertEqual(samples, [])
        connection.mav.request_data_stream_send.assert_called_once()
        connection.close.assert_called_once()

    def test_failure_log_copy_is_bounded_and_records_destination(self) -> None:
        with tempfile.TemporaryDirectory() as raw_dir:
            directory = Path(raw_dir)
            source = directory / "temporary.log"
            source.write_bytes(b"0123456789")
            output = directory / "result.json"
            retained = _retain_failure_logs({"gazebo": source}, output, 4)
            destination = directory / "result.gazebo.log"
            self.assertEqual(destination.read_bytes(), b"6789")
        self.assertEqual(retained, [{
            "process": "gazebo",
            "path": str(destination),
            "bytes": 4,
            "source_bytes": 10,
            "truncated": True,
        }])


class SmokeOrchestrationTests(unittest.TestCase):
    class FakeProcess:
        pass

    class RecordingSupervisor:
        instance = None

        def __init__(self) -> None:
            self.commands: list[tuple[str, list[str]]] = []
            self.names: list[str] = []
            SmokeOrchestrationTests.RecordingSupervisor.instance = self

        def start(
            self,
            name: str,
            command: list[str],
            cwd: Path,
            log: object,
        ) -> "SmokeOrchestrationTests.FakeProcess":
            self.commands.append((name, command))
            self.names.append(name)
            log.write(f"{name} output\n")
            log.flush()
            return SmokeOrchestrationTests.FakeProcess()

        def wait_until(
            self,
            name: str,
            process: "SmokeOrchestrationTests.FakeProcess",
            predicate: object,
            timeout_s: float,
        ) -> None:
            return None

        def cleanup(self, grace_s: float = 5.0) -> list[Check]:
            return [Check(f"cleanup.{name}", Status.PASS, None, "stopped") for name in sorted(self.names)]

    def _smoke_patches(self) -> tuple[object, object]:
        return (
            patch("echorescue.sim_integration.diagnose", return_value={"ready": True, "checks": []}),
            patch("echorescue.sim_integration.ProcessSupervisor", self.RecordingSupervisor),
        )

    def test_missing_telemetry_fails_but_cleans_up_and_retains_logs(self) -> None:
        config = load_config()
        with tempfile.TemporaryDirectory() as raw_dir:
            output = Path(raw_dir) / "smoke.json"
            diagnose_patch, supervisor_patch = self._smoke_patches()
            with (
                diagnose_patch,
                supervisor_patch,
                patch.dict(os.environ, {"ARDUPILOT_HOME": str(Path.cwd())}),
                patch("echorescue.sim_integration._mavlink_samples", return_value=[]),
            ):
                report = smoke(config, timeout_s=1.0, headless=True, output_path=output)
            retained_paths = [Path(item["path"]) for item in report["logs"]]
            self.assertTrue(all(path.is_file() for path in retained_paths))
        self.assertEqual(report["status"], Status.FAIL)
        sitl_health = next(item for item in report["checks"] if item["name"] == "health.ardupilot_sitl")
        self.assertIn("received 0", sitl_health["detail"])
        self.assertEqual(len(report["logs"]), 2)
        self.assertTrue(all(item["status"] == Status.PASS for item in report["cleanup"]))

    def test_success_uses_no_rebuild_and_does_not_retain_logs(self) -> None:
        config = load_config()
        diagnose_patch, supervisor_patch = self._smoke_patches()
        with (
            diagnose_patch,
            supervisor_patch,
            patch.dict(os.environ, {"ARDUPILOT_HOME": str(Path.cwd())}),
            patch(
                "echorescue.sim_integration._mavlink_samples",
                return_value=[{"time_boot_ms": 100}, {"time_boot_ms": 200}],
            ) as sample_probe,
        ):
            report = smoke(config, timeout_s=1.0, headless=True)
        self.assertEqual(report["status"], Status.PASS)
        self.assertEqual(report["logs"], [])
        sample_probe.assert_called_once_with("tcp:127.0.0.1:5760", 2, 1.0, 10)
        sitl_command = next(
            command
            for name, command in self.RecordingSupervisor.instance.commands
            if name == "ardupilot_sitl"
        )
        self.assertIn("--no-rebuild", sitl_command)

    def test_startup_timeout_cleans_up_and_references_bounded_log(self) -> None:
        class TimeoutSupervisor(self.RecordingSupervisor):
            def wait_until(
                self,
                name: str,
                process: "SmokeOrchestrationTests.FakeProcess",
                predicate: object,
                timeout_s: float,
            ) -> None:
                raise StartupTimeout(f"{name} readiness timed out after {timeout_s:.1f}s")

        config = load_config()
        config["logging"]["failure_log_max_bytes"] = 5
        with tempfile.TemporaryDirectory() as raw_dir:
            output = Path(raw_dir) / "timeout.json"
            with (
                patch("echorescue.sim_integration.diagnose", return_value={"ready": True, "checks": []}),
                patch("echorescue.sim_integration.ProcessSupervisor", TimeoutSupervisor),
                patch.dict(os.environ, {"ARDUPILOT_HOME": str(Path.cwd())}),
            ):
                report = smoke(config, timeout_s=0.25, headless=True, output_path=output)
            retained = Path(report["logs"][0]["path"])
            self.assertEqual(retained.stat().st_size, 5)
        self.assertEqual(report["status"], Status.FAIL)
        self.assertIn("timed out", report["checks"][0]["detail"])
        self.assertTrue(all(item["status"] == Status.PASS for item in report["cleanup"]))


class ProcessSupervisorTests(unittest.TestCase):
    def _sleep_command(self) -> list[str]:
        return [sys.executable, "-c", "import time; time.sleep(30)"]

    def test_startup_timeout_and_cleanup(self) -> None:
        supervisor = ProcessSupervisor()
        with tempfile.TemporaryDirectory() as raw_dir:
            with (Path(raw_dir) / "child.log").open("w") as log:
                process = supervisor.start("sleeper", self._sleep_command(), Path(raw_dir), log)
                with self.assertRaises(StartupTimeout):
                    supervisor.wait_until("sleeper", process, lambda: False, 0.15)
                results = supervisor.cleanup(grace_s=1.0)
        self.assertIsNotNone(process.poll())
        self.assertEqual(results[0].status, Status.PASS)

    def test_unexpected_child_exit(self) -> None:
        supervisor = ProcessSupervisor()
        with tempfile.TemporaryDirectory() as raw_dir:
            with (Path(raw_dir) / "child.log").open("w") as log:
                process = supervisor.start("short", [sys.executable, "-c", "raise SystemExit(7)"], Path(raw_dir), log)
                with self.assertRaises(UnexpectedProcessExit):
                    supervisor.wait_until("short", process, lambda: False, 2.0)
                results = supervisor.cleanup()
        self.assertEqual(process.returncode, 7)
        self.assertEqual(results[0].status, Status.PASS)

    @unittest.skipIf(os.name == "nt", "POSIX process-group descendant assertion")
    def test_cleanup_terminates_descendant_processes(self) -> None:
        supervisor = ProcessSupervisor()
        with tempfile.TemporaryDirectory() as raw_dir:
            directory = Path(raw_dir)
            child_pid_path = directory / "child.pid"
            parent_program = (
                "import pathlib, subprocess, sys, time; "
                "child=subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)']); "
                "pathlib.Path(sys.argv[1]).write_text(str(child.pid)); time.sleep(30)"
            )
            with (directory / "tree.log").open("w") as log:
                supervisor.start("tree", [sys.executable, "-c", parent_program, str(child_pid_path)], directory, log)
                deadline = time.monotonic() + 2.0
                while not child_pid_path.exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(child_pid_path.exists())
                child_pid = int(child_pid_path.read_text())
                results = supervisor.cleanup(grace_s=1.0)
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        self.assertEqual(results[0].status, Status.PASS)

    def test_interruption_always_runs_cleanup(self) -> None:
        config = load_config()

        class FakeProcess:
            pass

        class InterruptingSupervisor:
            instance = None

            def __init__(self) -> None:
                self.cleaned = False
                InterruptingSupervisor.instance = self

            def start(self, name: str, command: list[str], cwd: Path, log: object) -> FakeProcess:
                return FakeProcess()

            def wait_until(self, name: str, process: FakeProcess, predicate: object, timeout_s: float) -> None:
                raise KeyboardInterrupt

            def cleanup(self, grace_s: float = 5.0) -> list[Check]:
                self.cleaned = True
                return [Check("cleanup.gazebo", Status.PASS, None, "stopped")]

        with (
            patch("echorescue.sim_integration.diagnose", return_value={"ready": True, "checks": []}),
            patch("echorescue.sim_integration.ProcessSupervisor", InterruptingSupervisor),
            patch.dict(os.environ, {"ARDUPILOT_HOME": str(Path.cwd())}),
        ):
            report = smoke(config, timeout_s=1.0, headless=True)
        self.assertEqual(report["status"], Status.FAIL)
        self.assertTrue(InterruptingSupervisor.instance.cleaned)
        self.assertEqual(report["cleanup"][0]["status"], Status.PASS)


if __name__ == "__main__":
    unittest.main()
