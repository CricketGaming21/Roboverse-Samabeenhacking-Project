"""pyhulax — SIM drop-in replacement for the real pyhulax SDK.

Mission code does `import pyhulax` / `from pyhulax import DroneAPI` unchanged;
which pyhulax is on sys.path IS the sim/real swap (HULA_SIM_BUILD_PLAN.md §5.5).

Public surface: DroneAPI, Dola, pyhulax.core (types), pyhulax.video,
pyhulax.exceptions. Everything in _bridge.py and simcore/ is internal.
"""

from .api import DroneAPI
from .discovery import Dola
from .exceptions import LowBattery, NotReady, PyhulaxError, TelemetryUnavailable

__all__ = [
    "DroneAPI",
    "Dola",
    "PyhulaxError",
    "NotReady",
    "LowBattery",
    "TelemetryUnavailable",
]
