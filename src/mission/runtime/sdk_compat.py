"""SDK compatibility shim — ONE codebase for the sim SUBSET and the real SDK.

The sim implements only a subset of `DroneAPI` (docs/RECONCILIATION.md §1). The
real-SDK-only init/teardown calls (`set_app_mode`, `send_app_heartbeat`,
`set_velocity_level`, `stop_manual_control`, `disconnect`, `enable_battery_failsafe`,
`arm`/`disarm`) are routed through `hasattr` guards here so they are **no-ops on the
sim and real on hardware**. Mission code must NEVER call those methods directly.
"""

from __future__ import annotations

from typing import Optional


def _try(drone, name: str, *args) -> bool:
    """Call drone.<name>(*args) iff it exists. Returns True if it ran."""
    fn = getattr(drone, name, None)
    if callable(fn):
        fn(*args)
        return True
    return False


def prepare_manual_control(drone, velocity_level: Optional[int] = None) -> None:
    """Real-SDK init before manual control: app mode, arm, first heartbeat, velocity
    level. No-op on the sim (these attrs are ALL absent); real on hardware."""
    _try(drone, "set_app_mode", 1)
    _try(drone, "arm")
    _try(drone, "send_app_heartbeat")
    if velocity_level is not None:
        _try(drone, "set_velocity_level", velocity_level)


def send_heartbeat(drone) -> None:
    """Heartbeat tick (real SDK only); no-op on the sim."""
    _try(drone, "send_app_heartbeat")


def enable_battery_failsafe(drone, *args) -> bool:
    """Firmware battery failsafe if available; else the mission handles it (P11)."""
    return _try(drone, "enable_battery_failsafe", *args)


def release(drone) -> None:
    """Stop manual control, disarm + disconnect if those exist (real SDK); no-op on the
    sim. Landing is done via the public `land()` BEFORE this — release is teardown only."""
    _try(drone, "stop_manual_control")
    _try(drone, "disarm")
    _try(drone, "disconnect")
