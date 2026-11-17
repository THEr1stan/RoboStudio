"""Compatibility alias. New code uses robostudio.simulation.recovery."""

import sys

from .simulation import recovery as _implementation

sys.modules[__name__] = _implementation
