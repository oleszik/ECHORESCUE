#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -m echorescue.telemetry_integration \
  --config "${repo_root}/config/coordinate-frames-v0.14.2.json" "$@"
