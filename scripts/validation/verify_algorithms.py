"""Disclosed fixed-query matrix: all 13 methods on both robots plus new physical tasks."""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from robostudio.serialization import canonical
from robostudio.studio_api import save_report
from robostudio.studio_lab import FAMILIES
from robostudio.studio_runtime import run_experiment
from robostudio.studio_scene import default_scene

OBSTACLE = [{"xy": [0.43, 0.015], "size": [0.035, 0.045, 0.115], "yaw": 0}]


def bounded_lab(request):
    """Identical 10 s process deadline; includes startup, computation and serialization."""
    began = time.perf_counter()
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            "import json,sys; from robostudio.studio_lab import run_lab; "
            "print(json.dumps(run_lab(**json.load(sys.stdin))))",
        ],
        input=json.dumps(request),
        text=True,
        capture_output=True,
        check=True,
        timeout=10,
    )
    return json.loads(completed.stdout), time.perf_counter() - began


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/algorithms-validation"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []

    def record(row):
        rows.append(row)
        (args.output / "report.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
        print(json.dumps(row), flush=True)

    for robot in ("r6", "gantry"):
        for family, methods in FAMILIES.items():
            cfg = default_scene(robot)
            target = [0.42, -0.14, 0.23]
            if family == "planner":
                cfg["obstacles"] = OBSTACLE
                target[1] = 0.20
            for method in methods:
                result, elapsed = bounded_lab(
                    {"config": cfg, "family": family, "method": method, "target_m": target}
                )
                repeated, _ = bounded_lab(result["replay_request"])
                replayed = canonical(result) == canonical(repeated)
                if not replayed:
                    raise RuntimeError("lab replay mismatch")
                (args.output / (result["lab_id"] + ".json")).write_text(
                    json.dumps(result), encoding="utf-8"
                )
                r = result["result"]
                record(
                    {
                        "kind": "lab",
                        "robot": robot,
                        "family": family,
                        "method": method,
                        "id": result["lab_id"],
                        "wall_budget_s": 10,
                        "process_wall_s": elapsed,
                        "status": r["status"],
                        "replay_identical": True,
                        **{
                            k: r[k]
                            for k in (
                                "iterations",
                                "collision_checks",
                                "length_joint_space",
                                "position_error_m",
                                "tracking_rms_si",
                                "duration_s",
                                "physics_steps",
                            )
                            if k in r
                        },
                        "source_hash": result["source_hash"],
                    }
                )
        for key, value in (
            ("solver", "dls"),
            ("solver", "adaptive_dls"),
            ("planner", "direct"),
            ("planner", "rrt"),
            ("planner", "rrt_connect"),
            ("planner", "prm"),
            ("trajectory", "cubic"),
            ("trajectory", "trapezoidal"),
        ):
            cfg = default_scene(robot)
            cfg[key] = value
            if key == "planner":
                cfg["obstacles"] = OBSTACLE
            result = run_experiment(cfg)
            repeated = run_experiment(cfg, result["actions"], result["metrics"]["steps"])
            if canonical(result) != canonical(repeated):
                raise RuntimeError("physics replay mismatch")
            save_report(args.output, result)
            record(
                {
                    "kind": "physical_task",
                    "robot": robot,
                    "method": value,
                    "id": result["run_id"],
                    "status": result["status"],
                    "error": result["error"],
                    "metrics": result["metrics"],
                    "source_hash": result["source_hash"],
                    "replay_identical": True,
                }
            )
    print(json.dumps({"cases": len(rows), "identical_replays": len(rows)}))


if __name__ == "__main__":
    main()
