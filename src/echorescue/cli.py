import argparse
import json

from echorescue.config import SimulationConfig
from echorescue.simulation import Simulation
from echorescue.visualization import TerminalRenderer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the deterministic EchoRescue Phase 1 simulation."
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--width", type=int, default=21)
    parser.add_argument("--height", type=int, default=13)
    parser.add_argument("--sensor-range", type=int, default=4)
    parser.add_argument("--survivors", type=int, default=3)
    parser.add_argument("--survivor-range", type=int, default=3)
    parser.add_argument("--confirmation-observations", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=1_000)
    parser.add_argument(
        "--obstacle-density", type=float, default=0.08, metavar="FRACTION"
    )
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--delay", type=float, default=0.03)
    parser.add_argument("--show-ground-truth", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    if args.delay < 0:
        raise SystemExit("--delay must not be negative")
    config = SimulationConfig(
        width=args.width,
        height=args.height,
        seed=args.seed,
        obstacle_density=args.obstacle_density,
        sensor_range=args.sensor_range,
        survivor_count=args.survivors,
        survivor_sensor_range=args.survivor_range,
        survivor_confirmation_observations=args.confirmation_observations,
        max_steps=args.max_steps,
    )
    simulation = Simulation(config)
    renderer = None
    if args.visualize:
        renderer = TerminalRenderer(args.delay, args.show_ground_truth)
    result = simulation.run(renderer)
    print(json.dumps(result.to_dict(), indent=2))


if __name__ == "__main__":
    main()
