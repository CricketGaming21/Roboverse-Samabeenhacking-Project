"""Keyboard piloting of a drone via the REAL send_manual_control path.

Groundwork for manual flying in the live 3D-only viewer (scripts/live_view.py):
WASD = forward/back/left/right, R/F = up/down, Q/E = yaw (CCW/CW), arrow keys
= camera tilt. Held keys are polled at ~20 Hz and mapped to -1..+1 stick
inputs, then sent through DroneAPI.send_manual_control (Phase 19) — the same
control path a mission lock-on PID would drive. NOTHING is added to the
public surface; this is just a keyboard front-end onto the existing method.

keys_to_sticks / camera_tilt_dir are PURE functions (key-name set -> values),
so the mapping is testable without opening a window.
"""

import time

from pyhulax.core import CameraPitchMode

# movement key name -> (stick axis, sign). +rotate = CCW (left), matching
# send_manual_control's convention.
_MOVE_KEYS = {
    "w": ("forward", +1.0), "s": ("forward", -1.0),
    "d": ("right", +1.0), "a": ("right", -1.0),
    "r": ("up", +1.0), "f": ("up", -1.0),
    "q": ("rotate", +1.0), "e": ("rotate", -1.0),
}
# arrow keys tilt the camera: down = look further down (+pitch toward 90),
# up = look further up (-pitch toward 0 / horizon).
_CAM_KEYS = {"down": +1.0, "up": -1.0}

_AXES = ("forward", "right", "up", "rotate")


def keys_to_sticks(active):
    """Pure map: a set of held key names -> (forward, right, up, rotate),
    each clamped to [-1, 1]. Opposing keys cancel."""
    v = {a: 0.0 for a in _AXES}
    for k in active:
        if k in _MOVE_KEYS:
            axis, sign = _MOVE_KEYS[k]
            v[axis] += sign
    return tuple(max(-1.0, min(1.0, v[a])) for a in _AXES)


def camera_tilt_dir(active):
    """Pure map: held key names -> camera-tilt direction (+1 down, -1 up,
    0 none)."""
    return sum(_CAM_KEYS.get(k, 0.0) for k in active)


class KeyboardPilot:
    """Drives one DroneAPI via send_manual_control from held-key sets.

    The sim CONTAINS NO control logic here — this only maps keys to stick
    inputs and forwards them through the public method (no PID, no new API).
    """

    def __init__(self, drone_api, cam_tilt_rate_dps: float = 60.0):
        self._d = drone_api
        self._tilt_rate = float(cam_tilt_rate_dps)
        self._pitch = 0.0   # tracked camera pitch (0 forward .. 90 down)

    def apply(self, active_keys, dt: float = 0.05) -> bool:
        """One control frame from the held keys. Returns the
        send_manual_control result (False when the drone can't accept it)."""
        f, r, u, rot = keys_to_sticks(active_keys)
        ok = self._d.send_manual_control(forward=f, right=r, up=u, rotate=rot)
        tilt = camera_tilt_dir(active_keys)
        if tilt != 0.0:
            self._pitch = max(0.0, min(90.0,
                                       self._pitch + tilt * self._tilt_rate * dt))
            self._d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE,
                                     int(round(self._pitch)))
        return ok

    def stop(self) -> None:
        """Zero the sticks (hover/station-keep)."""
        self._d.send_manual_control()


# --------------------------------------------------------------------------- #
# pybullet keyboard polling (used by live_view; needs the GUI client)
# --------------------------------------------------------------------------- #

# pybullet keycode -> our key name. Letters are their ASCII codes; the arrow
# keys use pybullet's special constants (resolved lazily so importing this
# module never requires pybullet to be connected).
def _keycode_map():
    import pybullet as p
    m = {ord(c): c for c in "wasdrfqe"}
    m[p.B3G_UP_ARROW] = "up"
    m[p.B3G_DOWN_ARROW] = "down"
    m[p.B3G_LEFT_ARROW] = "left"
    m[p.B3G_RIGHT_ARROW] = "right"
    return m


def held_keys(client):
    """Set of our key names currently held down in the GUI window."""
    import pybullet as p
    codes = _keycode_map()
    events = p.getKeyboardEvents(physicsClientId=client)
    held = set()
    for code, state in events.items():
        if (state & p.KEY_IS_DOWN) and code in codes:
            held.add(codes[code])
    return held


def run_keyboard_loop(registry, drone_api, rate_hz: int = 20,
                      stop_predicate=None) -> None:
    """Poll the GUI keyboard at ~rate_hz and fly the drone. MAIN THREAD (the
    GUI client lives on the sim thread, but getKeyboardEvents is read-only)."""
    pilot = KeyboardPilot(drone_api)
    period = 1.0 / max(int(rate_hz), 1)
    while registry.is_alive():
        if stop_predicate is not None and stop_predicate():
            break
        try:
            keys = registry.run_on_sim_thread(
                lambda: held_keys(registry.client))
        except (RuntimeError, TimeoutError):
            break
        pilot.apply(keys, dt=period)
        time.sleep(period)
    pilot.stop()
