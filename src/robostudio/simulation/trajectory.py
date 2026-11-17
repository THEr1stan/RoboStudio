"""Normalized time laws shared by visualization and the physical reference generator."""

from ..core import ErrorCode
from ..core.contracts import finite
from ..core.errors import require

PROFILES = ("quintic", "cubic", "trapezoidal")


def time_profile(method: str, u: float) -> tuple[float, float, float]:
    """Return s, ds/du, d²s/du². Duration scaling is the caller's responsibility."""
    require(method in PROFILES, ErrorCode.INVALID_VALUE, "unknown time profile")
    finite(u, "normalized time")
    require(0 <= u <= 1, ErrorCode.INVALID_VALUE, "normalized time must be 0..1")
    if method == "quintic":
        return (
            u**3 * (10 - 15 * u + 6 * u * u),
            30 * u * u * (1 - u) ** 2,
            60 * u * (1 - 3 * u + 2 * u * u),
        )
    if method == "cubic":
        return 3 * u * u - 2 * u**3, 6 * u * (1 - u), 6 - 12 * u
    # Acceleration/deceleration occupy 1/4 each; area under velocity is exactly 1.
    a = 16 / 3
    if u < 0.25:
        return a * u * u / 2, a * u, a
    if u <= 0.75:
        return (u - 0.125) * 4 / 3, 4 / 3, 0.0
    return 1 - a * (1 - u) ** 2 / 2, a * (1 - u), -a
