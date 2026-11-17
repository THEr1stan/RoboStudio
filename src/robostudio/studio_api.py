"""Compatibility alias. New code uses robostudio.simulation.api."""

import sys

from .simulation import api as _implementation

sys.modules[__name__] = _implementation
