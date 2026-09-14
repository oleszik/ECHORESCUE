#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec python -m echorescue.frontier_integration \
  --config "${repo_root}/config/frontier-exploration-v0.16.0.json" "$@"
