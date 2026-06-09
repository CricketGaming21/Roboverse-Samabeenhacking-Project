"""Phase 27 acceptance test — live 3D-only viewer + keyboardcontrol (headless).

The live-3D mode is FREEZE-PROOF because it renders NO cameras (no referee
scanning, no insets, no drone-camera renders) — proven here by mode/state
without opening a GUI window (no display in CI). keyboardcontrol maps keys
to send_manual_control stick inputs (pure-function) and drives the REAL
Phase-19 control path. Public surface unchanged.
"""

import inspect

import pytest

from pyhulax import DroneAPI

from scripts import keyboardcontrol as kc
from scripts.live_view import live_registry
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    c.motion.realistic = False
    return c


# --------------------------------------------------------------------------- #
# Live-3D mode renders NO cameras (so it can't freeze)
# --------------------------------------------------------------------------- #

def test_live_mode_disables_all_camera_rendering():
    cfg = _cfg()
    cfg.scoring.enabled = True               # would normally start the referee
    # gui=False here only so the test needs no display; the camera-disable
    # guard is independent of the GUI connection (that's what we assert).
    reg = live_registry(cfg, gui=False)
    try:
        assert reg.cameras_enabled is False
        # the referee (the camera-scanning thread) is NOT started
        assert reg.referee is None
        # both camera-render entry points refuse — no getCameraImage at all
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(100)
        assert reg.render_camera(reg.drones[0]) is None
        assert reg.render_arena(320, 240) is None
    finally:
        shutdown_registry()


def test_cameras_enabled_default_true_unchanged():
    cfg = _cfg()
    cfg.scoring.enabled = True
    reg = get_registry(cfg)                   # default: cameras enabled
    try:
        assert reg.cameras_enabled is True
        assert reg.referee is not None        # referee runs as before
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(100)
        assert reg.render_arena(160, 120) is not None  # still renders
    finally:
        shutdown_registry()


def test_live_registry_requests_gui_and_no_cameras():
    # live_registry must wire gui + cameras_enabled=False (the freeze-proof
    # combination); proven by source, since opening a GUI needs a display.
    src = inspect.getsource(live_registry)
    assert "cameras_enabled=False" in src
    assert "gui=gui" in src


# --------------------------------------------------------------------------- #
# keyboardcontrol — pure key -> stick mapping
# --------------------------------------------------------------------------- #

def test_key_to_stick_mapping():
    # WASD = forward/back/left/right
    assert kc.keys_to_sticks({"w"}) == (1.0, 0.0, 0.0, 0.0)
    assert kc.keys_to_sticks({"s"}) == (-1.0, 0.0, 0.0, 0.0)
    assert kc.keys_to_sticks({"d"}) == (0.0, 1.0, 0.0, 0.0)
    assert kc.keys_to_sticks({"a"}) == (0.0, -1.0, 0.0, 0.0)
    # R/F = up/down
    assert kc.keys_to_sticks({"r"}) == (0.0, 0.0, 1.0, 0.0)
    assert kc.keys_to_sticks({"f"}) == (0.0, 0.0, -1.0, 0.0)
    # Q/E = yaw; +rotate = CCW (left)
    assert kc.keys_to_sticks({"q"}) == (0.0, 0.0, 0.0, 1.0)
    assert kc.keys_to_sticks({"e"}) == (0.0, 0.0, 0.0, -1.0)
    # combinations + opposing-keys cancellation + clamping
    assert kc.keys_to_sticks({"w", "d"}) == (1.0, 1.0, 0.0, 0.0)
    assert kc.keys_to_sticks({"w", "s"}) == (0.0, 0.0, 0.0, 0.0)
    assert kc.keys_to_sticks(set()) == (0.0, 0.0, 0.0, 0.0)


def test_camera_tilt_mapping():
    assert kc.camera_tilt_dir({"down"}) == 1.0     # look further down
    assert kc.camera_tilt_dir({"up"}) == -1.0      # look up toward horizon
    assert kc.camera_tilt_dir({"up", "down"}) == 0.0
    assert kc.camera_tilt_dir({"w"}) == 0.0


# --------------------------------------------------------------------------- #
# The pilot drives the REAL send_manual_control (no new surface)
# --------------------------------------------------------------------------- #

def test_pilot_uses_real_send_manual_control():
    # the control path is the public Phase-19 method, not a new API
    src = inspect.getsource(kc)
    assert "send_manual_control" in src
    assert "set_camera_angle" in src             # camera tilt via the API
    # KeyboardPilot.apply forwards to the drone's send_manual_control
    sig = inspect.signature(DroneAPI.send_manual_control)
    assert list(sig.parameters)[1:] == ["forward", "right", "up", "rotate"]


def test_pilot_flies_a_real_drone_via_manual_control():
    cfg = _cfg()
    cfg.scoring.enabled = False
    reg = live_registry(cfg, gui=False)          # cameras off, still flies
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(100)
        drone = reg.drones[0]
        pilot = kc.KeyboardPilot(d)
        assert pilot.apply({"w"}) is True        # forward stick accepted
        stick = reg.run_on_sim_thread(lambda: drone.manual_stick.copy())
        assert stick[0] == 1.0 and stick[1] == 0.0   # forward set, right 0
        # camera tilt: the down arrow tilts the (unused) camera via the API
        pilot.apply({"down"}, dt=0.5)
        pitch = reg.run_on_sim_thread(lambda: drone.camera_pitch_deg)
        assert pitch > 0.0                       # set_camera_angle was called
        pilot.stop()
        stick2 = reg.run_on_sim_thread(lambda: drone.manual_stick.copy())
        assert tuple(stick2) == (0.0, 0.0, 0.0, 0.0)   # released -> hover
    finally:
        shutdown_registry()


def test_no_new_public_surface():
    # live_view / keyboardcontrol add nothing to pyhulax/UWB
    import pyhulax
    import pyhulax.core
    from UWBParserThread import UWBParserThread
    public = (set(dir(pyhulax)) | set(dir(pyhulax.DroneAPI))
              | set(dir(pyhulax.core)) | set(dir(UWBParserThread)))
    leaked = [n for n in public if "keyboard" in n.lower()
              or "live_view" in n.lower() or "view3d" in n.lower()]
    assert leaked == []
