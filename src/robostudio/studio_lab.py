"""Compatibility alias. New code uses robostudio.simulation.lab."""

import sys

from .simulation import lab as _implementation

sys.modules[__name__] = _implementation
