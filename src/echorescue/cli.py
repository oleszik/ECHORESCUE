import argparse
import json

from echorescue.config import SimulationConfig
from echorescue.multi_simulation import MultiDroneSimulation
from echorescue.replay import record_simulation, write_replay
from echorescue.simulation import Simulation
from echorescue.visualization import TerminalRenderer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the deterministic EchoRescue Phase 1 simulation."
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--drones", type=int, choices=(1, 2), default=2)
    parser.add_argument(
        "--start-mode", choices=("adjacent", "shared-base"), default="adjacent"
    )
    parser.add_argument("--width", type=int, default=21)
    parser.add_argument("--height", type=int, default=13)
    parser.add_argument("--sensor-range", type=int, default=4)
    parser.add_argument("--survivors", type=int, default=3)
    parser.add_argument("--survivor-range", type=int, default=3)
    parser.add_argument("--confirmation-observations", type=int, default=2)
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
        "--relay-strategy", choices=("off", "adaptive"), default="off"
    )
    parser.add_argument("--relay-min-outage-steps", type=int, default=40)
    parser.add_argument("--relay-min-unsynced-cells", type=int, default=240)
    parser.add_argument("--relay-max-role-steps", type=int, default=16)
    parser.add_argument("--relay-cooldown-steps", type=int, default=8)
    parser.add_argument("--relay-max-deployments", type=int, default=1)
    parser.add_argument("--relay-energy-margin", type=float, default=5.0)
    parser.add_argument("--relay-min-benefit-ratio", type=float, default=20.0)
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
    parser.add_argument("--max-steps", type=int, default=1_000)
    parser.add_argument(
        "--obstacle-density", type=float, default=0.08, metavar="FRACTION"
    )
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--delay", type=float, default=0.03)
    parser.add_argument("--replay-out")
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
    start_positions = None
    if args.drones == 2 and args.start_mode == "shared-base":
        start_positions = ((1, 1), (1, 1))
    config = SimulationConfig(
        width=args.width,
        height=args.height,
        seed=args.seed,
        obstacle_density=args.obstacle_density,
        sensor_range=args.sensor_range,
        survivor_count=args.survivors,
        survivor_sensor_range=args.survivor_range,
        survivor_confirmation_observations=args.confirmation_observations,
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
        max_steps=args.max_steps,
    )
    simulation = (
        MultiDroneSimulation(config) if args.drones == 2 else Simulation(config)
    )
    renderer = None
    if args.visualize:
        renderer = TerminalRenderer(args.delay, args.show_ground_truth)
    if args.replay_out:
        if not isinstance(simulation, MultiDroneSimulation):
            raise SystemExit("--replay-out currently requires --drones 2")
        replay, result = record_simulation(simulation, renderer)
        write_replay(replay, args.replay_out)
    else:
        result = simulation.run(renderer)
    print(json.dumps(result.to_dict(), indent=2))


if __name__ == "__main__":
    main()
