"""SDK compatibility shim — ONE codebase for the sim SUBSET and the real SDK.

The sim implements only a subset of `DroneAPI` (docs/RECONCILIATION.md §1). The
real-SDK-only init/teardown calls (`set_app_mode`, `send_app_heartbeat`,
`set_velocity_level`, `stop_manual_control`, `disconnect`, `enable_battery_failsafe`,
`arm`/`disarm`, `set_video_resolution`) are routed through `hasattr` guards here so they
are **no-ops on the sim and real on hardware**. Mission code must NEVER call those methods
directly.

On real hardware, manual control needs a periodic `send_app_heartbeat` keep-alive — that
runs on a background thread started by `prepare_manual_control` and STOPPED by `release`.
On the sim (where `send_app_heartbeat` is absent) no thread is ever spawned, so the suite
stays green and deterministic.
"""

from __future__ import annotations

import threading
from typing import Dict


def _has(drone, name: str) -> bool:
    """True iff drone.<name> exists and is callable (the real-SDK branch of the guard)."""
    return callable(getattr(drone, name, None))


def _try(drone, name: str, *args) -> bool:
    """Call drone.<name>(*args) iff it exists. Returns True if it ran."""
    fn = getattr(drone, name, None)
    if callable(fn):
        fn(*args)
        return True
    return False


# --------------------------------------------------------------------------- #
# Heartbeat keep-alive (real SDK only) — a cleanly stoppable background thread
# --------------------------------------------------------------------------- #
class _Heartbeat:
    """Pings `send_app_heartbeat(user_mode)` at a fixed rate so the real SDK keeps the
    manual-control link alive. Daemon thread, cleanly stoppable. A dropped beat is swallowed
    (the next tick retries) — a heartbeat hiccup must never crash a worker."""

    def __init__(self, drone, hz: float, user_mode: int = 1):
        self._drone = drone
        self._user_mode = user_mode
        self._period = 1.0 / hz if hz > 0 else 0.1
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="sdk-compat-heartbeat", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(self._period):
            try:
                self._drone.send_app_heartbeat(self._user_mode)
            except Exception:
                pass

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout)

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()


_HEARTBEATS: Dict[int, _Heartbeat] = {}      # id(drone) -> live heartbeat
_HEARTBEAT_LOCK = threading.Lock()           # workers prepare/release from their own threads


def _start_heartbeat(drone, hz: float, user_mode: int = 1) -> None:
    """Start the heartbeat thread IFF the drone exposes `send_app_heartbeat` (real SDK).
    Sends the first beat synchronously (deterministic handshake), then runs the thread.
    No-op on the sim (method absent) — no thread spawned."""
    if not _has(drone, "send_app_heartbeat"):
        return                                       # sim: nothing to beat
    _stop_heartbeat(drone)                           # idempotent: replace any prior beat
    drone.send_app_heartbeat(user_mode)              # first beat now (don't wait one period)
    if hz <= 0:
        return                                       # single handshake beat only, no thread
    hb = _Heartbeat(drone, hz, user_mode)
    with _HEARTBEAT_LOCK:
        _HEARTBEATS[id(drone)] = hb
    hb.start()


def _stop_heartbeat(drone) -> None:
    with _HEARTBEAT_LOCK:
        hb = _HEARTBEATS.pop(id(drone), None)
    if hb is not None:
        hb.stop()


def heartbeat_running(drone) -> bool:
    """True iff a heartbeat thread is registered and alive for this drone (inspection/test)."""
    with _HEARTBEAT_LOCK:
        hb = _HEARTBEATS.get(id(drone))
    return bool(hb and hb.alive)


def _resolve_velocity_level(level):
    """Map a config velocity level to what `set_velocity_level` wants. `None` → None (skip);
    an int / VelocityLevel member passes through; a name (e.g. ``"MEDIUM"``) resolves via the
    `pyhulax` on the path. Unknown/unavailable → None (skip — never guess a faster band)."""
    if level is None:
        return None
    if isinstance(level, int):                       # int or IntEnum member → use as-is
        return level
    try:
        from pyhulax.core import VelocityLevel
        return VelocityLevel[str(level)]
    except Exception:
        return None


# --------------------------------------------------------------------------- #
# Public shim entry points
# --------------------------------------------------------------------------- #
def prepare_manual_control(drone, velocity_level=None, heartbeat_hz: float = 10.0) -> None:
    """Real-SDK init before manual control. Each step is hasattr-guarded → no-op on the sim,
    real on hardware: app mode → background heartbeat thread (`send_app_heartbeat` at
    `heartbeat_hz`) → velocity band → **arm LAST**. The heartbeat is stopped by `release`."""
    _try(drone, "set_app_mode", 1)
    _start_heartbeat(drone, heartbeat_hz)
    level = _resolve_velocity_level(velocity_level)
    if level is not None:
        _try(drone, "set_velocity_level", level)
    _try(drone, "arm")


def prepare_telemetry(drone) -> None:
    """Real-SDK init for READ-ONLY telemetry: app mode + one heartbeat so data flows.
    **NEVER arms, NEVER starts manual control, NEVER starts the heartbeat thread** — for
    the read-only bring-up check. No-op on the sim; real on hardware."""
    _try(drone, "set_app_mode", 1)
    _try(drone, "send_app_heartbeat")                # single handshake beat (no thread)


def send_heartbeat(drone) -> None:
    """One heartbeat tick (real SDK only); no-op on the sim. The continuous keep-alive is the
    `prepare_manual_control` thread — this is for a one-off manual ping."""
    _try(drone, "send_app_heartbeat")


def enable_battery_failsafe(drone, *args) -> bool:
    """Firmware battery failsafe if available; else the mission handles it (P11)."""
    return _try(drone, "enable_battery_failsafe", *args)


def start_video_stream(drone, resolution=None) -> None:
    """Enable the video stream. On real, drop to a LOW resolution FIRST (guarded) — three
    simultaneous streams at full res peg the C2 laptop. `set_video_resolution` is real-only
    (absent on the sim → skipped); `set_video_stream` is public on both, so it always runs."""
    res = resolution if resolution is not None else _default_video_resolution()
    if res is not None:
        _try(drone, "set_video_resolution", res)
    drone.set_video_stream(True)


def _default_video_resolution():
    try:
        from pyhulax.core import VideoResolution
        return VideoResolution.LOW
    except Exception:
        return None


def release(drone) -> None:
    """Teardown (guarded; no-op on the sim): STOP the heartbeat thread first, then stop manual
    control, disarm, disconnect. Landing is done via the public `land()` BEFORE release."""
    _stop_heartbeat(drone)
    _try(drone, "stop_manual_control")
    _try(drone, "disarm")
    _try(drone, "disconnect")
