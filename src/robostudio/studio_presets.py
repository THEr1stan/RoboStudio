"""Compatibility alias. New code uses robostudio.simulation.presets."""

import sys

from .simulation import presets as _implementation

sys.modules[__name__] = _implementation
