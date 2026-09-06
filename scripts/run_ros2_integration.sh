#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
source "${repo_root}/ros2_ws/install/setup.bash"
set -u
export PYTHONUNBUFFERED=1
ros_bin="${repo_root}/ros2_ws/install/echorescue_ros/lib/echorescue_ros"
sim_pid=""
mission_pid=""

cleanup() {
  if [[ -n "${mission_pid}" ]]; then
    kill -INT "${mission_pid}" 2>/dev/null || true
    wait "${mission_pid}" 2>/dev/null || true
  fi
  if [[ -n "${sim_pid}" ]]; then
    kill -INT "${sim_pid}" 2>/dev/null || true
    wait "${sim_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

assert_report() {
  local path="$1" expected_state="$2" expected_success="$3"
  python3 - "$path" "$expected_state" "$expected_success" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding="utf-8"))
assert payload["state"] == sys.argv[2], payload
assert payload["mission_success"] is (sys.argv[3] == "true"), payload
print(sys.argv[1], payload["state"], payload["commands_issued"], payload["survivors_confirmed"])
PY
}

run_launch() {
  local domain="$1" report="$2" replay="$3"
  shift 3
  rm -f "$report" "$replay"
  ROS_DOMAIN_ID="$domain" timeout 30 ros2 launch echorescue_ros closed_loop.launch.py \
    "report_out:=${report}" "replay_out:=${replay}" "$@" >/tmp/echorescue-launch-"${domain}".log 2>&1
}

"${repo_root}/.venv-ros2/bin/python" -m echorescue.closed_loop_demo \
  --report-out /tmp/echorescue-reference.json \
  --replay-out /tmp/echorescue-reference-replay.json >/tmp/echorescue-reference.log
run_launch 130 /tmp/echorescue-normal.json /tmp/echorescue-normal-replay.json
assert_report /tmp/echorescue-normal.json mission_completed true
python3 - <<'PY'
import json

direct = json.load(open('/tmp/echorescue-reference.json', encoding='utf-8'))
ros = json.load(open('/tmp/echorescue-normal.json', encoding='utf-8'))
commands = lambda report: [
    event['target'] for event in report['events'] if event['event'] == 'command_issued'
]
assert direct['mission_success'] and ros['mission_success']
assert direct['survivors_confirmed'] == ros['survivors_confirmed'] == 2
assert commands(direct) == commands(ros)
print('/tmp/echorescue-normal.json matches_direct_reference_command_sequence')
PY

run_launch 131 /tmp/echorescue-no-observation.json /tmp/echorescue-no-observation-replay.json \
  drop_observations_after:=3 data_timeout_s:=0.5
assert_report /tmp/echorescue-no-observation.json controlled_stop false

run_launch 132 /tmp/echorescue-no-state.json /tmp/echorescue-no-state-replay.json \
  drop_states_after:=3 data_timeout_s:=0.5
assert_report /tmp/echorescue-no-state.json controlled_stop false

run_launch 133 /tmp/echorescue-duplicates.json /tmp/echorescue-duplicates-replay.json \
  duplicate_messages:=true publish_delayed_messages:=true
assert_report /tmp/echorescue-duplicates.json mission_completed true
python3 - <<'PY'
import json
p = json.load(open('/tmp/echorescue-duplicates.json', encoding='utf-8'))
assert any(e['event'] == 'duplicate_observation_ignored' for e in p['events'])
assert any(e['event'] == 'stale_state_ignored' for e in p['events'])
PY

# Mission-process failure: the backend must stop on its independent monotonic
# heartbeat lease and must not execute anything after that point.
export ROS_DOMAIN_ID=134
"${ros_bin}/simulator_node" --ros-args -p command_lease_timeout_s:=0.5 \
  >/tmp/echorescue-process-loss-simulator.log 2>&1 &
sim_pid=$!
"${ros_bin}/mission_node" --ros-args -p session_id:=failure-session \
  -p report_out:=/tmp/unused-process-loss.json -p replay_out:=/tmp/unused-process-loss-replay.json \
  >/tmp/echorescue-process-loss-mission.log 2>&1 &
mission_pid=$!
for _ in $(seq 1 100); do
  grep -q 'move:3 completed' /tmp/echorescue-process-loss-mission.log && break
  sleep 0.05
done
kill -9 "$mission_pid" 2>/dev/null || true
wait "$mission_pid" 2>/dev/null || true
mission_pid=""
sleep 1
kill -INT "$sim_pid" 2>/dev/null || true
wait "$sim_pid" 2>/dev/null || true
sim_pid=""
grep -q 'controlled backend stop: mission heartbeat timeout' /tmp/echorescue-process-loss-simulator.log
echo "/tmp/echorescue-process-loss-simulator.log controlled_backend_stop"

# A new session may take ownership only after the old lease expires.  It
# synchronizes to the actual retained state; a stale old-session goal is then
# rejected without motion.
export ROS_DOMAIN_ID=135
"${ros_bin}/simulator_node" --ros-args -p command_lease_timeout_s:=0.5 \
  >/tmp/echorescue-restart-simulator.log 2>&1 &
sim_pid=$!
"${ros_bin}/mission_node" --ros-args -p session_id:=session-one \
  >/tmp/echorescue-restart-first.log 2>&1 &
mission_pid=$!
for _ in $(seq 1 100); do
  grep -q 'move:3 completed' /tmp/echorescue-restart-first.log && break
  sleep 0.05
done
kill -9 "$mission_pid" 2>/dev/null || true
wait "$mission_pid" 2>/dev/null || true
mission_pid=""
sleep 1
"${ros_bin}/mission_node" --ros-args -p session_id:=session-two \
  -p report_out:=/tmp/echorescue-restart.json -p replay_out:=/tmp/echorescue-restart-replay.json \
  >/tmp/echorescue-restart-second.log 2>&1
assert_report /tmp/echorescue-restart.json mission_completed true
set +e
ros2 action send_goal /echorescue/move_grid echorescue_interfaces/action/MoveGrid \
  "{session_id: session-one, agent_id: drone-1, command_id: stale-command, expected_state_sequence: 0, issued_at: {sec: 0, nanosec: 0}, valid_until: {sec: 2, nanosec: 0}, source_x: 1, source_y: 1, target_x: 2, target_y: 1}" \
  >/tmp/echorescue-stale-command.log 2>&1
set -e
sleep 0.2
kill -INT "$sim_pid" 2>/dev/null || true
wait "$sim_pid" 2>/dev/null || true
sim_pid=""
grep -q 'session_or_agent_mismatch' /tmp/echorescue-restart-simulator.log
echo "/tmp/echorescue-restart.json new_session_complete_old_command_rejected"

echo "ROS2_INTEGRATION_OK"
