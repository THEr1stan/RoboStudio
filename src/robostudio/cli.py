import argparse
import json
import sys
from pathlib import Path
from typing import NoReturn

from . import __version__
from .core import ErrorCode, RoboStudioError, RunStatus
from .demo import replay, run_demo
from .registry import default_registry, load_robot


class Parser(argparse.ArgumentParser):
    def error(self, message: str) -> NoReturn:
        raise RoboStudioError(ErrorCode.INVALID_VALUE, message, "arguments")


def build_parser() -> argparse.ArgumentParser:
    parser = Parser(prog="robostudio", description="RoboStudio AI — local robotics workbench")
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("list", help="List registered robots or plugin manifests")
    listing.add_argument("kind", choices=("robots", "plugins"))
    validate = commands.add_parser(
        "validate", help="Validate a local robot manifest and model hash"
    )
    validate.add_argument("kind", choices=("robot",))
    validate.add_argument("manifest", type=Path)
    demo = commands.add_parser("demo", help="Run deterministic simulation test double")
    demo.add_argument("kind", choices=("fake",))
    demo.add_argument("--seed", type=int, default=42)
    demo.add_argument("--steps", type=int, default=20)
    demo.add_argument("--physics-dt", type=float, default=0.01)
    demo.add_argument("--control-dt", type=float, default=0.02)
    demo.add_argument("--output", type=Path, default=Path("artifacts"))
    physics = commands.add_parser("physics", help="Run verified R6 MuJoCo gravity release")
    physics.add_argument("--seed", type=int, default=42)
    physics.add_argument("--output", type=Path, default=Path("artifacts"))
    tracking = commands.add_parser("track", help="Solve and track a bounded R6 pose target")
    tracking.add_argument("--preset", choices=("reach", "sweep", "lift"), default="reach")
    tracking.add_argument("--controller", choices=("pd_bias", "pd"), default="pd_bias")
    tracking.add_argument("--output", type=Path, default=Path("artifacts"))
    jacobian = commands.add_parser("jacobian", help="Verify current R6 TCP Jacobian and velocity")
    jacobian.add_argument("--q", type=float, nargs=6, default=[0.0] * 6, help="J1..J6 radians")
    jacobian.add_argument(
        "--qvel",
        type=float,
        nargs=6,
        default=[1.0, 0, 0, 0, 0, 0],
        help="J1..J6 rad/s, bounded by +/- pi",
    )
    experiment = commands.add_parser("experiment", help="Replay and verify a saved experiment")
    experiment.add_argument("action", choices=("replay",))
    experiment.add_argument("run", help="Run ID or path to run.json")
    experiment.add_argument("--output", type=Path, default=Path("artifacts"))
    workbench = commands.add_parser("workbench", help="Open the local browser workbench")
    workbench.add_argument("--port", type=int, default=8765)
    workbench.add_argument("--output", type=Path, default=Path("artifacts"))
    workbench.add_argument("--open", action="store_true", help="Open the default browser")
    return parser


def main(argv: list[str] | None = None) -> int:
    try:
        args = build_parser().parse_args(argv)
        if args.command == "workbench":
            from .core.errors import require
            from .workbench import serve

            require(0 <= args.port <= 65535, ErrorCode.INVALID_VALUE, "invalid port", "port")
            serve(args.port, args.output, args.open)
            return 0
        output: object
        if args.command == "jacobian":
            from .jacobian import analyze_pose

            output = analyze_pose(tuple(args.q), tuple(args.qvel))
        elif args.command == "list":
            registry = default_registry()
            if args.kind == "robots":
                output = [
                    {
                        "robot_id": r.robot_id,
                        "family": r.family,
                        "capabilities": sorted(r.capabilities.values),
                    }
                    for r in registry.robots
                ]
            else:
                output = [
                    {"plugin_id": p.plugin_id, "provides_capability": p.provides_capability}
                    for p in registry.plugins
                ]
        elif args.command == "validate":
            robot = load_robot(args.manifest)
            output = {
                "valid": True,
                "robot_id": robot.robot_id,
                "robot_hash": robot.robot_hash,
                "model_hash": robot.model.sha256,
            }
        else:
            if args.command == "track":
                from .tracking import preset_config, run_tracking

                config = preset_config(args.preset)
                config["controller"] = args.controller
                run = run_tracking(config, args.output)
                print(
                    json.dumps(
                        {k: v for k, v in run.items() if k != "frames"},
                        ensure_ascii=False,
                        allow_nan=False,
                    )
                )
                return 0 if run["result"]["status"] == "SUCCEEDED" else 1
            elif args.command == "physics":
                from .arm_experiment import run_arm

                run = run_arm(args.seed, args.output)
                print(
                    json.dumps(
                        {key: value for key, value in run.items() if key != "frames"},
                        ensure_ascii=False,
                        allow_nan=False,
                    )
                )
                return 0 if run["result"]["status"] == "SUCCEEDED" else 1
            elif args.command == "demo":
                result = run_demo(
                    seed=args.seed,
                    output_dir=args.output,
                    steps=args.steps,
                    physics_dt_s=args.physics_dt,
                    control_dt_s=args.control_dt,
                )
            else:
                path = Path(args.run)
                if args.run.startswith("studio_") and path.name == args.run:
                    from .simulation.api import StudioAPI

                    output = StudioAPI(args.output).post("replay", {"run_id": args.run})
                    print(json.dumps(output, ensure_ascii=False))
                    return 0
                if args.run.startswith("cell_") and path.name == args.run:
                    from .workcell import handle_request

                    output = handle_request("replay", {"run_id": args.run}, args.output)
                    print(json.dumps(output, ensure_ascii=False))
                    return 0
                if path.name == args.run and args.run.startswith(("fake_", "arm_", "track_")):
                    path = args.output / args.run / "run.json"
                result = replay(path, output_dir=args.output)
            output = result.as_dict()
            if result.status == RunStatus.FAILED:
                print(json.dumps(output, ensure_ascii=False, allow_nan=False))
                return 1
        print(json.dumps(output, ensure_ascii=False, allow_nan=False))
        return 0
    except RoboStudioError as error:
        print(json.dumps({"error": error.as_dict()}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
