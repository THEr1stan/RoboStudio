"""Fixed M1 acceptance corpus and matched PD baseline; writes measured JSON only."""

import argparse
import json
import math
import random
from pathlib import Path
from time import perf_counter

from robostudio.backends.mujoco_backend import MuJoCoBackend
from robostudio.core import Pose
from robostudio.core.motion import IKProblem
from robostudio.motion import HOME, DampedLeastSquares, R6Kinematics, quaternion_from_matrix
from robostudio.registry import default_registry
from robostudio.tracking import PRESETS, preset_config, run_tracking


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/m1-completion"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    robot = default_registry().robot("r6_compact")
    model, solver = R6Kinematics(robot), DampedLeastSquares()
    rng = random.Random(20261114)
    backend = MuJoCoBackend()
    backend.load(robot)
    backend.reset(42)
    cases = []
    try:
        for i in range(50):
            target_q = tuple(rng.uniform(lo * 0.8, hi * 0.8) for lo, hi in model.limits)
            backend.set_configuration(target_q)
            target_scene = backend.scene()
            target = Pose(
                tuple(target_scene["tcp"]),
                quaternion_from_matrix(target_scene["rotation"]),
                "world",
            )
            start = perf_counter()
            result = solver.solve(
                model, IKProblem(target, HOME, robot.robot_hash, robot.model.sha256)
            )
            elapsed = perf_counter() - start
            backend.set_configuration(result.q_rad)
            actual = backend.scene()
            # Independent engine rotation residual via trace, not solver pose_error.
            relative_trace = sum(
                actual["rotation"][a][b] * target_scene["rotation"][a][b]
                for a in range(3)
                for b in range(3)
            )
            angle = math.acos(min(1, max(-1, (relative_trace - 1) / 2)))
            distance = math.dist(actual["tcp"], target.position_m)
            if result.status == "CONVERGED":
                assert distance <= 1e-4 and angle <= 1e-3
            cases.append(
                {
                    "case": i,
                    "goal_q_rad": target_q,
                    "status": result.status,
                    "iterations": result.iterations,
                    "attempts": result.attempts,
                    "position_error_m": distance,
                    "orientation_error_rad": angle,
                    "wall_time_s": elapsed,
                }
            )
    finally:
        backend.close()
    comparisons = []
    for preset in PRESETS:
        pair = {"preset": preset[0]}
        for controller in ("pd", "pd_bias"):
            config = preset_config(preset[0])
            config["controller"] = controller
            run = run_tracking(config, args.output)
            pair[controller] = run["result"]
        assert pair["pd"]["metrics"]["steps"] == pair["pd_bias"]["metrics"]["steps"]
        assert pair["pd_bias"]["status"] == "SUCCEEDED"
        comparisons.append(pair)
    report = {
        "seed": 20261114,
        "case_count": 50,
        "ik_converged": sum(c["status"] == "CONVERGED" for c in cases),
        "ik_cases": cases,
        "controller_comparison": comparisons,
        "scope": "fixed original model; bounded local solver; not a global reachability proof",
    }
    (args.output / "acceptance.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {"cases": 50, "converged": report["ik_converged"], "comparisons": comparisons}, indent=2
        )
    )


if __name__ == "__main__":
    main()
