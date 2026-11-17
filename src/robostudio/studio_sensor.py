"""Compatibility alias. New code uses robostudio.simulation.sensor."""

import sys

from .simulation import sensor as _implementation

sys.modules[__name__] = _implementation
