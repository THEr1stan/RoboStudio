"""Compatibility alias. New code uses robostudio.simulation.documents."""

import sys

from .simulation import documents as _implementation

sys.modules[__name__] = _implementation
