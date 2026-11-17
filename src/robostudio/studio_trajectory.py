"""Compatibility alias. New code uses robostudio.simulation.trajectory."""

import sys

from .simulation import trajectory as _implementation

sys.modules[__name__] = _implementation
