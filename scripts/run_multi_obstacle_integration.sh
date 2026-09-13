#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec python -m echorescue.multi_obstacle_integration \
  --scenarios "${repo_root}/config/multi-obstacle-scenarios-v0.15.2.json" "$@"
