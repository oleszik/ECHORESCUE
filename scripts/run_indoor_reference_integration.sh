#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
exec python -m echorescue.indoor_integration --config "${repo_root}/config/indoor-reference-v0.14.5.json" "$@"
