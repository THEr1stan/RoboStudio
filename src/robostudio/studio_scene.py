"""Compatibility alias. New code uses robostudio.simulation.scene."""

import sys

from .simulation import scene as _implementation

sys.modules[__name__] = _implementation
