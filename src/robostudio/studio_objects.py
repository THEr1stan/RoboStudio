"""Compatibility alias. New code uses robostudio.simulation.objects."""

import sys

from .simulation import objects as _implementation

sys.modules[__name__] = _implementation
