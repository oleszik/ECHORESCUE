import argparse
import json

from echorescue.config import MAX_DRONE_COUNT, SimulationConfig
from echorescue.multi_simulation import (
    MultiDroneSimulation,
    MultiSimulationResult,
)
from echorescue.replay import record_simulation, write_replay
from echorescue.visualization import TerminalRenderer


def failure_spec(value: str) -> tuple[str, int]:
    try:
        drone_id, raw_step = value.rsplit(":", 1)
        step = int(raw_step)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "failure must use DRONE_ID:STEP, for example drone-2:20"
        ) from error
    if (
        not drone_id.startswith("drone-")
        or not drone_id.removeprefix("drone-").isdigit()
        or int(drone_id.removeprefix("drone-")) < 1
        or step < 0
    ):
        raise argparse.ArgumentTypeError(
            "failure must use a positive drone-N ID and a non-negative step"
        )
    return drone_id, step


def obstacle_spec(value: str) -> tuple[int, int, int]:
    try:
        raw_position, raw_step = value.rsplit(":", 1)
        raw_x, raw_y = raw_position.split(",", 1)
        x, y, step = int(raw_x), int(raw_y), int(raw_step)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "obstacle must use X,Y:STEP, for example 5,7:20"
        ) from error
    if x < 0 or y < 0 or step < 1:
        raise argparse.ArgumentTypeError(
            "obstacle coordinates must be non-negative and step positive"
        )
    return x, y, step


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the deterministic EchoRescue search-and-rescue simulation."
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--drones", type=int, choices=range(1, MAX_DRONE_COUNT + 1), default=2
    )
    parser.add_argument(
        "--start-mode", choices=("adjacent", "shared-base"), default="adjacent"
    )
    parser.add_argument("--width", type=int, default=21)
    parser.add_argument("--height", type=int, default=13)
    parser.add_argument("--sensor-range", type=int, default=4)
    parser.add_argument("--survivors", type=int, default=3)
    parser.add_argument("--survivor-range", type=int, default=3)
    parser.add_argument(
        "--survivor-sensor",
        choices=("visual", "thermal"),
        default="visual",
        help="select one isolated Survivor perception channel",
    )
    parser.add_argument("--thermal-survivor-range", type=int, default=3)
    parser.add_argument("--thermal-detection-probability", type=float, default=0.6)
    parser.add_argument("--thermal-smoke-attenuation", type=float, default=0.15)
    parser.add_argument("--confirmation-observations", type=int, default=2)
    parser.add_argument(
        "--perception-noise",
        choices=("off", "moderate"),
        default="off",
        help="opt in to deterministic Survivor perception noise",
    )
    parser.add_argument("--battery-capacity", type=float, default=220.0)
    parser.add_argument("--movement-energy", type=float, default=1.0)
    parser.add_argument("--sensor-energy", type=float, default=0.05)
    parser.add_argument("--energy-reserve", type=float, default=20.0)
    parser.add_argument("--wait-energy", type=float, default=0.05)
    parser.add_argument("--communication-range", type=int, default=8)
    parser.add_argument("--proximity-range", type=int, default=2)
    parser.add_argument("--intent-reservation-steps", type=int, choices=(2, 3), default=3)
    parser.add_argument("--motion-intent-ttl", type=int, default=4)
    parser.add_argument("--deadlock-wait-threshold", type=int, default=3)
    parser.add_argument(
        "--disable-distributed-deconfliction", action="store_true"
    )
    parser.add_argument(
        "--relay-strategy",
        choices=("off", "adaptive", "network-aware"),
        default="off",
    )
    parser.add_argument("--relay-min-outage-steps", type=int, default=40)
    parser.add_argument("--relay-min-unsynced-cells", type=int, default=240)
    parser.add_argument("--relay-max-role-steps", type=int, default=16)
    parser.add_argument("--relay-cooldown-steps", type=int, default=8)
    parser.add_argument("--relay-max-deployments", type=int, default=1)
    parser.add_argument("--relay-energy-margin", type=float, default=5.0)
    parser.add_argument("--relay-min-benefit-ratio", type=float, default=20.0)
    parser.add_argument("--network-relay-utility-threshold", type=float, default=25.0)
    parser.add_argument("--network-relay-max-backlog", type=int, default=144)
    parser.add_argument("--network-relay-max-hops", type=int, default=2)
    parser.add_argument("--network-relay-hysteresis", type=int, default=3)
    parser.add_argument("--network-relay-recent-map-age", type=int, default=8)
    parser.add_argument("--network-relay-map-delta-limit", type=int, default=24)
    parser.add_argument("--network-relay-min-outage", type=int, default=8)
    parser.add_argument(
        "--network-profile", choices=("ideal", "constrained"), default="ideal"
    )
    parser.add_argument("--network-latency-steps", type=int, default=1)
    parser.add_argument("--network-packet-loss", type=float, default=0.05)
    parser.add_argument("--network-link-capacity", type=int, default=36)
    parser.add_argument("--network-fragment-size", type=int, default=12)
    parser.add_argument("--network-map-ttl", type=int, default=256)
    parser.add_argument("--network-survivor-ttl", type=int, default=128)
    parser.add_argument("--network-fairness-age", type=int, default=8)
    parser.add_argument("--network-backlog-warning", type=int, default=24)
    parser.add_argument("--final-sync-max-steps", type=int, default=128)
    parser.add_argument(
        "--knowledge-mode",
        choices=("shared", "shadow", "local"),
        default="shared",
    )
    parser.add_argument("--disable-base-knowledge-store", action="store_true")
    parser.add_argument(
        "--inject-failure",
        action="append",
        default=[],
        type=failure_spec,
        metavar="DRONE_ID:STEP",
        help="deterministically fail a drone at a simulation step (repeatable)",
    )
    parser.add_argument(
        "--smoke-profile",
        choices=("off", "moderate"),
        default="off",
        help="opt in to deterministic smoke-degraded survivor perception",
    )
    parser.add_argument(
        "--dynamic-obstacles",
        choices=("off", "moderate"),
        default="off",
        help="opt in to deterministic persistent path blockages",
    )
    parser.add_argument(
        "--inject-obstacle",
        action="append",
        default=[],
        type=obstacle_spec,
        metavar="X,Y:STEP",
        help="persistently block an initially-free cell at a mission step",
    )
    parser.add_argument("--max-steps", type=int, default=1_000)
    parser.add_argument(
        "--obstacle-density", type=float, default=0.08, metavar="FRACTION"
    )
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--delay", type=float, default=0.03)
    parser.add_argument("--replay-out")
    parser.add_argument(
        "--replay-debug-smoke",
        action="store_true",
        help="include a debug-only ground-truth smoke layer in replay output",
    )
    parser.add_argument(
        "--show-ground-truth",
        action="store_true",
        help="deprecated compatibility flag; hidden ground truth remains concealed",
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    if args.delay < 0:
        raise SystemExit("--delay must not be negative")
    if args.replay_debug_smoke and not args.replay_out:
        raise SystemExit("--replay-debug-smoke requires --replay-out")
    if args.replay_debug_smoke and args.smoke_profile == "off":
        raise SystemExit(
            "--replay-debug-smoke requires an active --smoke-profile"
        )
    start_positions = None
    if args.start_mode == "shared-base":
        start_positions = tuple((1, 1) for _ in range(args.drones))
    config = SimulationConfig(
        width=args.width,
        height=args.height,
        seed=args.seed,
        obstacle_density=args.obstacle_density,
        sensor_range=args.sensor_range,
        survivor_count=args.survivors,
        survivor_sensor_range=args.survivor_range,
        survivor_sensor=args.survivor_sensor,
        thermal_survivor_sensor_range=args.thermal_survivor_range,
        thermal_detection_probability=args.thermal_detection_probability,
        thermal_smoke_attenuation=args.thermal_smoke_attenuation,
        survivor_confirmation_observations=args.confirmation_observations,
        perception_noise=args.perception_noise,
        battery_capacity=args.battery_capacity,
        movement_energy_cost=args.movement_energy,
        sensor_energy_cost=args.sensor_energy,
        energy_safety_reserve=args.energy_reserve,
        drone_count=args.drones,
        drone_start_positions=start_positions,
        wait_energy_cost=args.wait_energy,
        communication_range=args.communication_range,
        proximity_sensor_range=args.proximity_range,
        intent_reservation_steps=args.intent_reservation_steps,
        motion_intent_ttl=args.motion_intent_ttl,
        deadlock_wait_threshold=args.deadlock_wait_threshold,
        distributed_deconfliction_enabled=(
            not args.disable_distributed_deconfliction
        ),
        relay_strategy=args.relay_strategy,
        relay_min_outage_steps=args.relay_min_outage_steps,
        relay_min_unsynced_cells=args.relay_min_unsynced_cells,
        relay_max_role_steps=args.relay_max_role_steps,
        relay_cooldown_steps=args.relay_cooldown_steps,
        relay_max_deployments=args.relay_max_deployments,
        relay_energy_margin=args.relay_energy_margin,
        relay_min_benefit_ratio=args.relay_min_benefit_ratio,
        network_relay_utility_threshold=args.network_relay_utility_threshold,
        network_relay_max_backlog_units=args.network_relay_max_backlog,
        network_relay_max_hops=args.network_relay_max_hops,
        network_relay_hysteresis_steps=args.network_relay_hysteresis,
        network_relay_recent_map_age_steps=args.network_relay_recent_map_age,
        network_relay_map_delta_limit=args.network_relay_map_delta_limit,
        network_relay_min_outage_steps=args.network_relay_min_outage,
        network_profile=args.network_profile,
        network_latency_steps=args.network_latency_steps,
        network_packet_loss_rate=args.network_packet_loss,
        network_link_capacity_units=args.network_link_capacity,
        network_max_fragment_units=args.network_fragment_size,
        network_map_ttl=args.network_map_ttl,
        network_survivor_ttl=args.network_survivor_ttl,
        network_fairness_age_steps=args.network_fairness_age,
        network_backlog_warning_threshold=args.network_backlog_warning,
        final_sync_max_steps=args.final_sync_max_steps,
        knowledge_mode=args.knowledge_mode,
        base_knowledge_store_enabled=not args.disable_base_knowledge_store,
        failure_schedule=tuple(sorted(args.inject_failure, key=lambda item: item[1])),
        dynamic_obstacles=args.dynamic_obstacles,
        dynamic_obstacle_schedule=tuple(
            sorted(args.inject_obstacle, key=lambda item: item[2])
        ),
        smoke_profile=args.smoke_profile,
        max_steps=args.max_steps,
    )
    simulation = MultiDroneSimulation(config)
    renderer = None
    if args.visualize:
        renderer = TerminalRenderer(args.delay, args.show_ground_truth)
    result: MultiSimulationResult
    if args.replay_out:
        replay, result = record_simulation(
            simulation,
            renderer,
            include_debug_smoke=args.replay_debug_smoke,
        )
        write_replay(replay, args.replay_out)
    else:
        result = simulation.run(renderer)
    print(json.dumps(result.to_dict(), indent=2))


if __name__ == "__main__":
    main()
