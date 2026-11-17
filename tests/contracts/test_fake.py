from dataclasses import replace

import pytest

from robostudio.backends.fake import FakeBackend
from robostudio.core import ErrorCode, RoboStudioError, RobotCommand, SimulationBackend


def command_for(robot, state, values=(1.0, -0.5)):
    return RobotCommand(
        robot_id=robot.robot_id,
        robot_hash=robot.robot_hash,
        model_hash=robot.model.sha256,
        frame_id=robot.frame_id,
        time_s=state.time_s,
        effort_si=values,
    )


def test_lifecycle(robot):
    backend = FakeBackend()
    assert isinstance(backend, SimulationBackend)
    with pytest.raises(RoboStudioError) as error:
        backend.observe()
    assert error.value.code == ErrorCode.INVALID_LIFECYCLE
    backend.load(robot)
    with pytest.raises(RoboStudioError):
        backend.step(0.01)
    backend.reset(42)
    before = backend.observe()
    backend.apply(command_for(robot, before))
    backend.step(0.01)
    after = backend.observe()
    assert before.time_s == 0.0 and after.time_s == 0.01
    # ctrl[0] maps to joint2, ctrl[1] to joint1; explicit mapping is observable.
    assert after.qvel_si == pytest.approx((-0.005, 0.01))
    backend.reset(42)
    assert backend.observe() == before
    backend.close()
    backend.close()
    with pytest.raises(RoboStudioError):
        backend.reset(42)


@pytest.mark.parametrize("failure", ["dimension", "frame", "time", "model"])
def test_invalid_command_does_not_mutate_state(robot, failure):
    backend = FakeBackend()
    backend.load(robot)
    backend.reset(42)
    before = backend.observe()
    command = command_for(robot, before)
    fields = {
        "dimension": {"effort_si": (1.0,)},
        "frame": {"frame_id": "tool"},
        "time": {"time_s": 1.0},
        "model": {"model_hash": "0" * 64},
    }
    with pytest.raises(RoboStudioError):
        backend.apply(replace(command, **fields[failure]))
    assert backend.observe() == before


def test_deterministic_and_seed_sensitive(robot):
    def trajectory(seed):
        backend = FakeBackend()
        backend.load(robot)
        backend.reset(seed)
        samples = [backend.observe()]
        for _ in range(10):
            backend.apply(command_for(robot, samples[-1]))
            backend.step(0.01)
            samples.append(backend.observe())
        backend.close()
        return samples

    assert trajectory(42) == trajectory(42)
    assert trajectory(42) != trajectory(43)
