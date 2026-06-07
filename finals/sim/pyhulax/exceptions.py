"""Custom exceptions for the (simulated) pyhulax SDK.

These mirror the error surface the mission code is allowed to depend on
(HULA_SIM_BUILD_PLAN.md §4.7).
"""


class PyhulaxError(Exception):
    """Base exception for all pyhulax errors."""


class NotReady(PyhulaxError):
    """Raised when a command is issued while not connected / not flying."""


class LowBattery(PyhulaxError):
    """Raised when a motion command is issued below the battery threshold."""


class TelemetryUnavailable(PyhulaxError):
    """Raised when telemetry is requested before any data exists."""
