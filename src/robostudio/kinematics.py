"""Independent geometric reference for the ORIGINAL R6 Compact, SI/radian/world.

Deliberately does not derive axes, offsets or transforms from the physics engine.
"""

import math
import random
from typing import Any, Protocol

from .core import ErrorCode
from .core.contracts import vector
from .core.errors import require

AXES = (2, 1, 1, 0, 1, 0)
OFFSETS = ((0, 0, 0.22), (0.32, 0, 0), (0.28, 0, 0), (0.07, 0, 0), (0.07, 0, 0), (0.1, 0, 0))
LIMITS = tuple((-math.radians(v), math.radians(v)) for v in (180, 120, 140, 180, 120, 180))


def forward_kinematics(q: tuple[float, ...]) -> dict[str, Any]:
    vector(q, "q_rad", 6)
    rotation = [[float(i == j) for j in range(3)] for i in range(3)]
    point = [0.0, 0.0, 0.0]
    points = [point.copy()]
    axes = []
    for axis, offset, angle in zip(AXES, OFFSETS, q, strict=True):
        axes.append([row[axis] for row in rotation])
        c, s = math.cos(angle), math.sin(angle)
        turn = [[float(i == j) for j in range(3)] for i in range(3)]
        i, j = (axis + 1) % 3, (axis + 2) % 3
        turn[i][i] = turn[j][j] = c
        turn[i][j], turn[j][i] = -s, s
        rotation = [
            [sum(rotation[a][k] * turn[k][b] for k in range(3)) for b in range(3)] for a in range(3)
        ]
        point = [point[i] + sum(rotation[i][j] * offset[j] for j in range(3)) for i in range(3)]
        points.append(point.copy())
    return {"points": points, "jointAxes": axes, "tcp": point, "rotation": rotation}


class KinematicProbe(Protocol):
    def set_configuration(self, q: tuple[float, ...]) -> None: ...
    def scene(self) -> dict[str, Any]: ...


def verify_model(backend: KinematicProbe, seed: int = 42, samples: int = 100) -> dict[str, Any]:
    require(
        type(samples) is int and 1 <= samples <= 1000,
        ErrorCode.INVALID_VALUE,
        "samples must be in [1, 1000]",
    )
    require(type(seed) is int and 0 <= seed < 2**32, ErrorCode.INVALID_VALUE, "invalid seed")
    rng = random.Random(seed)
    configurations = [(0.0,) * 6] + [
        tuple(lo + rng.random() * (hi - lo) for lo, hi in LIMITS) for _ in range(samples)
    ]
    max_position = max_rotation = max_axis = 0.0
    for q in configurations:
        backend.set_configuration(q)
        expected, actual = forward_kinematics(q), backend.scene()
        for a, b in zip(actual["points"], expected["points"], strict=True):
            max_position = max(max_position, math.dist(a, b))
        for key in ("rotation", "jointAxes"):
            error = max(
                abs(a - b)
                for row_a, row_b in zip(actual[key], expected[key], strict=True)
                for a, b in zip(row_a, row_b, strict=True)
            )
            if key == "rotation":
                max_rotation = max(max_rotation, error)
            else:
                max_axis = max(max_axis, error)
    require(
        max(max_position, max_rotation, max_axis) <= 1e-10,
        ErrorCode.MODEL_MISMATCH,
        "independent FK differs from physics model",
    )
    return {
        "configurations": len(configurations),
        "seed": seed,
        "max_position_error_m": max_position,
        "max_rotation_error": max_rotation,
        "max_axis_error": max_axis,
        "tolerance": 1e-10,
    }
