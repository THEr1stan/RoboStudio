"""Small, reproducible fake experiment and integrity-checked replay."""

import hashlib
import platform
import tempfile
from dataclasses import asdict, dataclass
from importlib.resources import files
from math import isclose, sqrt
from pathlib import Path
from typing import cast

from . import __version__
from .backends.fake import FakeBackend
from .core import ErrorCode, RoboStudioError, RobotCommand, RunResult, RunStatus
from .core.contracts import finite
from .core.errors import require
from .registry import default_registry
from .registry.manifests import record
from .serialization import canonical, digest, read_json

SCENARIO = {
    "scenario_id": "fake_effort_v1",
    "robot_id": "fake_arm2",
    "effort_si": [1.0, -0.5],
    "scope": "test-double-only",
}


@dataclass(frozen=True)
class DemoConfig:
    seed: int = 42
    steps: int = 20
    physics_dt_s: float = 0.01
    control_dt_s: float = 0.02

    def __post_init__(self) -> None:
        require(
            type(self.seed) is int and 0 <= self.seed < 2**32,
            ErrorCode.INVALID_VALUE,
            "seed must be a uint32",
            "seed",
        )
        require(
            type(self.steps) is int and 1 <= self.steps <= 10000,
            ErrorCode.INVALID_VALUE,
            "steps must be in [1, 10000]",
            "steps",
        )
        for name in ("physics_dt_s", "control_dt_s"):
            finite(getattr(self, name), name)
        require(
            0 < self.physics_dt_s <= 0.1 and self.physics_dt_s <= self.control_dt_s <= 1.0,
            ErrorCode.INVALID_VALUE,
            "expected 0 < physics_dt <= 0.1 and physics_dt <= control_dt <= 1.0",
            "dt",
        )
        ratio = self.control_dt_s / self.physics_dt_s
        finite(ratio, "control_dt/physics_dt")
        require(
            isclose(ratio, round(ratio), rel_tol=0, abs_tol=1e-9),
            ErrorCode.INVALID_VALUE,
            "control_dt must be a multiple of physics_dt",
            "dt",
        )


def source_hash() -> str:
    root = files("robostudio")

    def walk(relative: str = "") -> list[tuple[str, str]]:
        entries: list[tuple[str, str]] = []
        for item in sorted(root.joinpath(relative).iterdir(), key=lambda p: p.name):
            name = f"{relative}/{item.name}".lstrip("/")
            if item.is_dir() and item.name != "__pycache__":
                entries.extend(walk(name))
            elif item.is_file() and item.name.endswith(
                (".py", ".json", ".xml", ".html", ".css", ".js", ".svg")
            ):
                entries.append(
                    (name, hashlib.sha256(item.read_bytes().replace(b"\r\n", b"\n")).hexdigest())
                )
        return entries

    return digest(walk())


def _build_run(config: DemoConfig) -> tuple[RunResult, dict[str, object], list[dict[str, object]]]:
    robot = default_registry().robot("fake_arm2")
    config_dict = asdict(config)
    identity = {
        "config_hash": digest(config_dict),
        "scenario_hash": digest(SCENARIO),
        "source_hash": source_hash(),
        "robot_hash": robot.robot_hash,
        "model_hash": robot.model.sha256,
    }
    environment = {
        "python": platform.python_version(),
        "implementation": platform.python_implementation(),
        "robostudio": __version__,
    }
    run_id = "fake_" + digest({**identity, "environment": environment})[:24]
    backend = FakeBackend()
    samples: list[dict[str, object]] = []
    failure: RoboStudioError | None = None
    try:
        backend.load(robot)
        backend.reset(config.seed)
        samples.append(asdict(backend.observe()))
        control_stride = round(config.control_dt_s / config.physics_dt_s)
        for step in range(config.steps):
            if step % control_stride == 0:
                state = backend.observe()
                backend.apply(
                    RobotCommand(
                        robot_id=robot.robot_id,
                        robot_hash=robot.robot_hash,
                        model_hash=robot.model.sha256,
                        frame_id=robot.frame_id,
                        time_s=state.time_s,
                        effort_si=(1.0, -0.5),
                    )
                )
            backend.step(config.physics_dt_s)
            samples.append(asdict(backend.observe()))
    except RoboStudioError as error:
        failure = error
    finally:
        backend.close()
    metrics: dict[str, float] = {"steps": max(len(samples) - 1, 0)}
    if samples:
        metrics["sim_time_s"] = cast(float, samples[-1]["time_s"])
        metrics["final_speed_norm_rad_s"] = sqrt(
            sum(v * v for v in cast(tuple[float, ...], samples[-1]["qvel_si"]))
        )
        metrics["position_rms_rad"] = sqrt(
            sum(v * v for sample in samples for v in cast(tuple[float, ...], sample["qpos_si"]))
            / (len(samples) * robot.qpos_size)
        )
    result = RunResult(
        run_id=run_id,
        scenario_id=str(SCENARIO["scenario_id"]),
        **identity,
        seed=config.seed,
        status=RunStatus.FAILED if failure else RunStatus.SUCCEEDED,
        metrics=metrics,
        error_code=failure.code if failure else None,
    )
    document: dict[str, object] = {
        "schema_version": "v1alpha1",
        "result": result.as_dict(),
        "config": config_dict,
        "scenario": SCENARIO,
        "environment": environment,
        "scope": "Simulation-only / fake test double; no L0/L1/L2 paper claim",
        "observations_hash": digest(samples),
        "error": failure.as_dict() if failure else None,
        "replay_command": "robostudio experiment replay run.json --output ..",
        "replay_working_directory": "directory containing this run.json",
    }
    return result, document, samples


def _save(
    output_dir: Path, run_id: str, document: dict[str, object], samples: list[dict[str, object]]
) -> None:
    payloads = {
        "run.json": canonical(document) + "\n",
        "observations.json": canonical(samples) + "\n",
    }
    destination = output_dir / run_id
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            for name, payload in payloads.items():
                require(
                    (destination / name).is_file()
                    and (destination / name).read_text("utf-8") == payload,
                    ErrorCode.INTEGRITY_ERROR,
                    "existing artifact differs; use another output directory",
                    str(destination / name),
                )
            return
        with tempfile.TemporaryDirectory(prefix=".run-", dir=output_dir) as temporary:
            staging = Path(temporary) / run_id
            staging.mkdir()
            for name, payload in payloads.items():
                (staging / name).write_text(payload, encoding="utf-8", newline="\n")
            staging.rename(destination)
    except OSError as error:
        raise RoboStudioError(ErrorCode.IO_ERROR, str(error), str(destination)) from error


def run_demo(
    seed: int = 42,
    output_dir: Path = Path("artifacts"),
    steps: int = 20,
    physics_dt_s: float = 0.01,
    control_dt_s: float = 0.02,
) -> RunResult:
    result, document, samples = _build_run(DemoConfig(seed, steps, physics_dt_s, control_dt_s))
    _save(output_dir, result.run_id, document, samples)
    return result


def replay(path: Path, output_dir: Path = Path("artifacts")) -> RunResult:
    document = record(read_json(path), "run")
    if document.get("schema_version") == "tracking-run-v1":
        from .tracking import replay_tracking

        return replay_tracking(path, output_dir)
    if document.get("schema_version") == "arm-run-v1":
        from .arm_experiment import replay_arm

        return replay_arm(path, output_dir)
    config = record(
        document.get("config"), "config", {"seed", "steps", "physics_dt_s", "control_dt_s"}
    )
    result, expected, samples = _build_run(
        DemoConfig(
            seed=cast(int, config["seed"]),
            steps=cast(int, config["steps"]),
            physics_dt_s=cast(float, config["physics_dt_s"]),
            control_dt_s=cast(float, config["control_dt_s"]),
        )
    )
    observations = read_json(path.parent / "observations.json")
    require(
        canonical(document) == canonical(expected)
        and canonical(observations) == canonical(samples),
        ErrorCode.INTEGRITY_ERROR,
        "replay integrity mismatch: source, environment, inputs or observations changed",
    )
    _save(output_dir, result.run_id, expected, samples)
    return result
