#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec python -m echorescue.sensor_replanning_integration \
  --config "${repo_root}/config/sensor-replanning-v0.15.1.json" "$@"
