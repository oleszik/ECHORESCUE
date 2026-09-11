#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec python -m echorescue.planner_flight_integration \
  --config "${repo_root}/config/planner-flight-v0.15.0.json" "$@"
