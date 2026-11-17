"""Compatibility alias. New code uses robostudio.simulation.planning."""

import sys

from .simulation import planning as _implementation

sys.modules[__name__] = _implementation
