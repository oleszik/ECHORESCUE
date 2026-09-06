"""Run the v0.13 loop without ROS as a deterministic reference transport."""

import argparse

from echorescue.bridge_artifacts import build_closed_loop_replay, write_json
from echorescue.bridge_contracts import BridgeHealth, BridgeHeartbeat
from echorescue.closed_loop import (
    ClosedLoopMission,
    ClosedLoopSimulatorBackend,
    MissionBridgeState,
    closed_loop_config,
)
from echorescue.config import SimulationConfig


def run_reference(
    config: SimulationConfig, *, session_id: str = "reference-session"
) -> tuple[ClosedLoopMission, ClosedLoopSimulatorBackend]:
    now = 1_000_000_000
    backend = ClosedLoopSimulatorBackend(config)
    mission = ClosedLoopMission(session_id, "drone-1", closed_loop_config(config))
    heartbeat_sequence = 1
    while mission.state not in {
        MissionBridgeState.COMPLETED,
        MissionBridgeState.CONTROLLED_STOP,
    }:
        mission_heartbeat = BridgeHeartbeat(
            session_id,
            "mission",
            heartbeat_sequence,
            backend.sim_time,
            BridgeHealth.READY,
        )
        new_session = backend.active_session != session_id
        backend.receive_mission_heartbeat(mission_heartbeat)
        mission.receive_backend_heartbeat(backend.heartbeat(), now)
        if new_session:
            mission.receive_observation(backend.observation(), now)
            mission.receive_state(backend.state_report(), now)
        command = mission.tick(now)
        if command is not None:
            result = backend.execute(command)
            mission.receive_result(result)
            mission.receive_observation(backend.observation(), now)
            mission.receive_state(backend.state_report(), now)
        heartbeat_sequence += 1
        now += 1_000_000
        if heartbeat_sequence > config.max_steps + 2:
            break
    return mission, backend


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--width", type=int, default=11)
    parser.add_argument("--height", type=int, default=9)
    parser.add_argument("--survivors", type=int, default=2)
    parser.add_argument("--report-out", default="replays/v0.13-reference-report.json")
    parser.add_argument("--replay-out", default="replays/v0.13-reference-replay.json")
    args = parser.parse_args()
    config = SimulationConfig(
        width=args.width,
        height=args.height,
        seed=args.seed,
        obstacle_density=0.04,
        survivor_count=args.survivors,
        battery_capacity=300.0,
        drone_count=1,
    )
    mission, backend = run_reference(config)
    report = {**mission.report(), "shield_interventions": backend.shield_interventions}
    write_json(report, args.report_out)
    write_json(build_closed_loop_replay(mission), args.replay_out)
    print(report)


if __name__ == "__main__":
    main()
