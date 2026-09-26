#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
report="${ECHORESCUE_PORTFOLIO_REPORT:-/tmp/echorescue-portfolio-demo.json}"
exec "${repo_root}/scripts/run_planner_flight_integration.sh" \
  --config "${repo_root}/config/planner-flight-v0.15.0.json" \
  --output "${report}" \
  smoke \
  --graphical \
  --gui-config "${repo_root}/config/portfolio-gui.config" \
  "$@"
