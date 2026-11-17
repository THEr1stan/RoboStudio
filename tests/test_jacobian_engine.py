import pytest

pytest.importorskip("mujoco", reason="optional physics extra is not installed")

from robostudio.backends.mujoco_backend import MuJoCoBackend
from robostudio.core import RoboStudioError
from robostudio.jacobian import analyze_pose, verify_jacobian
from robostudio.registry import default_registry


@pytest.fixture
def backend():
    instance = MuJoCoBackend()
    instance.load(default_registry().robot("r6_compact"))
    instance.reset(42)
    yield instance
    instance.close()


def test_jacobian_query_is_read_only_and_copied(backend):
    backend.step(0.002)
    state, scene = backend.observe(), backend.scene()
    matrix = backend.tcp_jacobian()
    assert backend.observe() == state and backend.scene() == scene
    assert len(matrix) == 6 and all(len(row) == 6 for row in matrix)
    backend.step(0.002)
    assert backend.tcp_jacobian() != matrix
    with pytest.raises(RoboStudioError, match="INVALID_LIFECYCLE"):
        MuJoCoBackend().tcp_jacobian()


def test_engine_difference_and_analytic_agree(backend):
    report = verify_jacobian(backend)
    assert report["configurations"] == 105
    assert report["max_engine_error"] < 1e-12
    assert report["max_fd_error"] < 1e-8
    assert report["frame_id"] == "world"


@pytest.mark.parametrize("corruption", ["sign", "column", "nan"])
def test_verification_detects_bad_column_or_sign(backend, monkeypatch, corruption):
    original = backend.tcp_jacobian

    def corrupted():
        matrix = original()
        if corruption == "sign":
            return tuple((-row[0],) + row[1:] for row in matrix)
        if corruption == "column":
            return tuple((row[1], row[0]) + row[2:] for row in matrix)
        return tuple((float("nan"),) + row[1:] for row in matrix)

    monkeypatch.setattr(backend, "tcp_jacobian", corrupted)
    with pytest.raises(RoboStudioError, match="INVALID_VALUE|MODEL_MISMATCH"):
        verify_jacobian(backend, samples=1)


def test_verification_detects_tcp_offset_error():
    import hashlib
    from dataclasses import replace
    from importlib.resources import files

    xml = files("robostudio").joinpath("data/r6_compact.xml").read_bytes()
    changed = xml.replace(b'name="tcp" pos="0.1 0 0"', b'name="tcp" pos="0.12 0 0"')
    assert changed != xml
    robot = default_registry().robot("r6_compact")
    robot = replace(robot, model=replace(robot.model, sha256=hashlib.sha256(changed).hexdigest()))
    backend = MuJoCoBackend(changed)
    try:
        backend.load(robot)
        backend.reset(42)
        with pytest.raises(RoboStudioError, match="MODEL_MISMATCH"):
            verify_jacobian(backend, samples=1)
    finally:
        backend.close()


def test_inspection_returns_reproducible_identity_and_si_velocity():
    data = analyze_pose((0,) * 6, (1, 0, 0, 0, 0, 0))
    assert data["twist_si"] == pytest.approx([0, 0.84, 0, 0, 0, 1])
    assert data["frame_id"] == "world" and len(data["model_hash"]) == 64
    assert data["verification"]["max_engine_error"] < 1e-12
    assert data["jacobian"][2][4] == pytest.approx(-0.17)


@pytest.mark.parametrize(
    "q,rate",
    [
        ((100,) * 6, (0,) * 6),
        ((0,) * 6, (10,) * 6),
        ((0,) * 6, (float("inf"),) * 6),
        ((0,) * 5, (0,) * 6),
    ],
)
def test_invalid_pose_or_rate_rejected(q, rate):
    with pytest.raises(RoboStudioError):
        analyze_pose(q, rate)
