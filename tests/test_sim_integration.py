import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from echorescue.sim_integration import (
    Check,
    ProcessSupervisor,
    StartupTimeout,
    Status,
    UnexpectedProcessExit,
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

    def test_telemetry_requires_a_changing_sample(self) -> None:
        self.assertFalse(telemetry_progressed([{"time_boot_ms": 1}, {"time_boot_ms": 1}]))
        self.assertTrue(telemetry_progressed([{"time_boot_ms": 1}, {"time_boot_ms": 2}]))


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
