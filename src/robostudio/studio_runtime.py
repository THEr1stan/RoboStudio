"""Compatibility alias. New code uses robostudio.simulation.runtime."""

import sys

from .simulation import runtime as _implementation

sys.modules[__name__] = _implementation
