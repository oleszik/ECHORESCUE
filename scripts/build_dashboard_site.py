"""Build the replay dashboard as a self-contained static site."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from echorescue.dashboard import (  # noqa: E402
    ASSET_DIRECTORY,
    DEFAULT_REPLAY,
    SCENARIOS,
    _project_summary,
    _repository_file,
    _scenario_public_entry,
)


def _copy_json(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)


def _write_json(payload: object, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")


def build(output: Path) -> None:
    if output.exists():
        shutil.rmtree(output)
    shutil.copytree(ASSET_DIRECTORY, output)

    _copy_json(_repository_file(DEFAULT_REPLAY), output / "replay.json")
    default_benchmark = SCENARIOS[0].get("benchmark")
    if not default_benchmark:
        raise RuntimeError("The default dashboard scenario requires a benchmark artifact")
    _copy_json(_repository_file(str(default_benchmark)), output / "benchmark.json")
    _write_json(
        {"default": "multi-agent", "scenarios": [_scenario_public_entry(item) for item in SCENARIOS]},
        output / "api" / "scenarios.json",
    )
    _write_json(_project_summary(), output / "api" / "project-summary.json")

    for scenario in SCENARIOS:
        scenario_root = output / "api" / "scenarios" / str(scenario["id"])
        for artifact_type in ("replay", "benchmark"):
            relative_path = scenario.get(artifact_type)
            if relative_path:
                _copy_json(
                    _repository_file(str(relative_path)),
                    scenario_root / f"{artifact_type}.json",
                )


if __name__ == "__main__":
    build(REPOSITORY_ROOT / "dist")
