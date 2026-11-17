"""Compatibility alias. New code uses robostudio.simulation.world."""

import sys

from .simulation import world as _implementation

sys.modules[__name__] = _implementation
