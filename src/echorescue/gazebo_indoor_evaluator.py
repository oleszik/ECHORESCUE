"""Independent Gazebo evidence collector for the v0.14.5 indoor run.

This process observes simulator topics only.  It has no ROS, MAVLink, or
mission-node dependency and cannot provide control inputs.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import queue
import subprocess
import threading
from time import monotonic, sleep
from typing import Any, Sequence

from echorescue.indoor_evaluation import (
    IndoorRunEvaluator, load_indoor_config, parse_contact_message,
    parse_pose_message, serialize_indoor_report,
)


def _json_stream(
    topic: str, process: subprocess.Popen[str], messages: queue.Queue[tuple[str, dict[str, Any]]],
) -> None:
    decoder = json.JSONDecoder()
    buffer = ""
    assert process.stdout is not None
    for chunk in process.stdout:
        buffer += chunk
        while buffer:
            stripped = buffer.lstrip()
            try:
                decoded, end = decoder.raw_decode(stripped)
            except json.JSONDecodeError:
                break
            if isinstance(decoded, dict):
                messages.put((topic, decoded))
            buffer = stripped[end:]


def _topic_list() -> set[str]:
    result = subprocess.run(
        ["gz", "topic", "-l"], capture_output=True, text=True,
        check=False, timeout=10,
    )
    if result.returncode != 0:
        raise RuntimeError(f"cannot list Gazebo topics: {result.stderr.strip()}")
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def evaluate(
    config_path: Path, output_path: Path, stop_path: Path,
    startup_timeout_s: float,
) -> bool:
    _, config = load_indoor_config(config_path)
    deadline = monotonic() + startup_timeout_s
    available: set[str] = set()
    required = {config.pose_topic, *config.contact_topics}
    while monotonic() < deadline:
        available = _topic_list()
        if required <= available:
            break
        sleep(0.2)
    if not required <= available:
        missing = sorted(required - available)
        report = IndoorRunEvaluator(config).report()
        report["detail"] = f"required Gazebo topics unavailable: {missing}"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(serialize_indoor_report(report), encoding="utf-8")
        return False

    evaluator = IndoorRunEvaluator(config)
    evaluator.set_available_topics(sorted(available))
    messages: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
    processes: list[tuple[str, subprocess.Popen[str]]] = []
    latest_position = None
    latest_simulation_time = 0.0
    try:
        for topic in sorted(required):
            process = subprocess.Popen(
                ["gz", "topic", "-e", "--json-output", "-t", topic],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                start_new_session=True,
            )
            processes.append((topic, process))
            threading.Thread(
                target=_json_stream, args=(topic, process, messages), daemon=True,
            ).start()
        collision_deadline: float | None = None
        while not stop_path.exists():
            if collision_deadline is not None and monotonic() >= collision_deadline:
                break
            try:
                topic, message = messages.get(timeout=0.2)
            except queue.Empty:
                if any(process.poll() is not None for _, process in processes):
                    raise RuntimeError("Gazebo topic observer exited unexpectedly")
                continue
            if topic == config.pose_topic:
                parsed = parse_pose_message(message, config.model_name)
                if parsed is None:
                    continue
                timestamp, latest_position = parsed
                latest_simulation_time = timestamp if timestamp is not None else latest_simulation_time + config.sample_interval_s
                evaluator.observe_pose(latest_position, latest_simulation_time)
            else:
                timestamp, pairs = parse_contact_message(message)
                entity = next(item for item in config.entities if item.contact_topic == topic)
                contact_time = timestamp if timestamp is not None else latest_simulation_time
                if (
                    entity.classification == "floor"
                    and (
                        entity.name in evaluator.contact_summary
                        or latest_position is None
                        or abs(contact_time - latest_simulation_time) > 0.2
                    )
                ):
                    # Independent topic readers are not globally ordered. Floor
                    # contact is evaluated only against a time-correlated pose;
                    # the pose-based envelope still covers every sample.
                    continue
                evaluator.observe_contacts(
                    entity.name, pairs,
                    contact_time,
                    latest_position,
                )
            if evaluator.collision_detected and collision_deadline is None:
                # Preserve prompt failure while giving independently configured
                    # contact sensors a short interval to record the same event.
                collision_deadline = monotonic() + 0.5
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        report = evaluator.report()
        report["detail"] = f"Gazebo evidence collection failed: {error}"
    else:
        report = evaluator.report()
        report["detail"] = (
            "collision evidence caused immediate evaluation failure"
            if evaluator.collision_detected else
            "independent Gazebo trajectory and contact evaluation completed"
        )
    finally:
        for _, process in processes:
            if process.poll() is None:
                process.terminate()
        for _, process in processes:
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(serialize_indoor_report(report), encoding="utf-8")
    return report["status"] == "PASS"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Observe v0.14.5 Gazebo collision and route evidence")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.startup_timeout <= 0:
        raise SystemExit("--startup-timeout must be positive")
    return 0 if evaluate(args.config, args.output, args.stop_file, args.startup_timeout) else 1


if __name__ == "__main__":
    raise SystemExit(main())
