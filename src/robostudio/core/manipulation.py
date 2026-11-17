"""SI/world contracts for variable-DOF manipulation kinematics. Standard library only."""

from typing import Protocol

from .contracts import Pose


class ManipulationProbe(Protocol):
    """A private probe. Evaluation must not modify the active physics state."""

    limits: list[tuple[float, ...]]
    configuration: tuple[float, ...]

    def evaluate(self, q: tuple[float, ...]) -> tuple[Pose, tuple[tuple[float, ...], ...]]: ...


class ManipulationKinematics(Protocol):
    capabilities: frozenset[str]

    def probe(self) -> ManipulationProbe: ...


class CollisionQuery(Protocol):
    """A pure query against an isolated scene snapshot; true means collision-free."""

    def __call__(self, q: list[float]) -> bool: ...
