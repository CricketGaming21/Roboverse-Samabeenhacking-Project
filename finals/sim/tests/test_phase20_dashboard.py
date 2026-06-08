"""Phase 20 acceptance test — command dashboard + boolean proximity overlay.

Both are observer-only views reading DebugProbe/simcore — never pyhulax,
nothing added to the public surface. The dashboard renders per-drone
telemetry / command-or-stick / barrier flags / scan status and mutates
nothing; the overlay is plain boolean directional (no angle anywhere).
"""

import inspect
import time

import pybullet as p
import pytest

from pyhulax import DroneAPI
from pyhulax.core import Direction

from scripts.dashboard import CommandDashboard, render_dashboard
from simcore import frames, viz
from simcore.config import load_config
from simcore.debug import DebugProbe
from simcore.registry import get_registry, shutdown_registry


def _cfg(rtf=10.0):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = rtf
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    c.motion.realistic = False
    return c


def _connect(cfg, index=0) -> DroneAPI:
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


# --------------------------------------------------------------------------- #
# Dashboard content + blocking vs manual mode
# --------------------------------------------------------------------------- #

def test_dashboard_renders_per_drone_fields():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg, 0)
        d.takeoff(100)
        dash = CommandDashboard(reg)
        text = dash.snapshot_text()
        assert "HULA COMMAND DASHBOARD" in text
        for i in range(len(cfg.drones.units)):
            assert f"drone {i}" in text
        # telemetry fields present for drone 0
        assert "UWB(n,e)" in text and "drift" in text
        assert "alt" in text and "hdg" in text and "batt" in text
        # barrier indicators (the five flags) + scan status line
        assert "BARRIER" in text
        assert all(f"{ltr}[" in text for ltr in ("F", "B", "L", "R", "D"))
        assert "SCAN" in text
    finally:
        shutdown_registry()


def test_dashboard_shows_move_to_goal_then_stick_inputs():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg, 0)
        d.takeoff(100)
        # blocking mode: a move_to goal with progress
        d.move_to(0, 200, 100, blocking=False)
        text = render_dashboard(DebugProbe(reg).snapshot())
        assert "CMD     move_to" in text
        assert "m to go" in text

        # manual mode: the live stick inputs replace the goal line
        d.send_manual_control(forward=0.5, rotate=0.3)
        text = render_dashboard(DebugProbe(reg).snapshot())
        assert "MANUAL" in text
        assert "FOR+0.50" in text and "ROT+0.30" in text
        d.send_manual_control()
    finally:
        shutdown_registry()


def test_dashboard_barrier_indicators_reflect_flags():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        def _box():
            pos = frames.arena_to_world(cfg, 2.8, 1.1, 1.0)
            col = p.createCollisionShape(p.GEOM_BOX,
                                         halfExtents=[0.6, 0.2, 1.0],
                                         physicsClientId=reg.client)
            return p.createMultiBody(0, col, basePosition=pos,
                                     physicsClientId=reg.client)
        reg.run_on_sim_thread(_box)
        d = _connect(cfg, 0)
        d.takeoff(100)
        probe = DebugProbe(reg)
        assert "F[ ]" in render_dashboard(probe.snapshot())  # clear ahead
        d.move_to(0, 150, 100)                  # n=2.1: face 0.5 m ahead, in range
        text = render_dashboard(probe.snapshot())
        assert "F[X]" in text                                # forward flag set
        # the dashboard indicator matches get_obstacles exactly
        assert d.get_obstacles().forward is True
    finally:
        shutdown_registry()


def test_dashboard_updates_over_ticks():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg, 0)
        d.takeoff(100)
        dash = CommandDashboard(reg)
        t1 = dash.snapshot_text().splitlines()[0]
        time.sleep(0.3)
        t2 = dash.snapshot_text().splitlines()[0]
        assert t1 != t2                       # the sim-time header advanced
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Observer-only boundaries
# --------------------------------------------------------------------------- #

def test_dashboard_is_read_only_and_uses_no_pyhulax():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg, 0)
        d.takeoff(100)  # crisp mode + no active goal => state is frozen unless
        d.hover(0.3)    # the dashboard mutates it
        pose0, yaw0 = reg.drone_world_pose(0)
        dash = CommandDashboard(reg)
        for _ in range(5):
            dash.snapshot_text()
        pose1, yaw1 = reg.drone_world_pose(0)
        # rendering mutated NOTHING: an idle drone's true pose is untouched
        assert pose0 == pose1 and yaw0 == yaw1
        assert reg.run_on_sim_thread(lambda: reg.drones[0].goal) is None
        # the dashboard imports simcore only — never pyhulax (check the
        # actual import lines, not the explanatory docstring prose)
        import scripts.dashboard as dashmod
        imports = [ln for ln in inspect.getsource(dashmod).splitlines()
                   if ln.lstrip().startswith(("import ", "from "))]
        assert any("simcore" in ln for ln in imports)
        assert not any("pyhulax" in ln for ln in imports)
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Boolean directional overlay — no angle anywhere
# --------------------------------------------------------------------------- #

def test_overlay_is_boolean_directional_no_angle():
    # the wedge/band/angle machinery is gone from viz entirely
    src = inspect.getsource(viz)
    for forbidden in ("Wedge", "cone_deg", "proximity_band", "_PROX_LABELS",
                      "_BAND_COLORS", "_RED_FRACTION"):
        assert forbidden not in src, f"angle/wedge machinery left in viz: {forbidden}"
    # the overlay helper takes boolean flags, not distances/angles
    sig = inspect.signature(viz.TopDownView._draw_proximity)
    assert "flags" in sig.parameters
    assert "cone_deg" not in sig.parameters and "angle" not in sig.parameters


def test_overlay_sample_is_five_booleans():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg, 0)
        d.takeoff(100)
        view = viz.TopDownView(reg)
        view.sample()
        prox = view._latest[0][7]
        assert set(prox) == {"forward", "back", "left", "right", "down"}
        assert all(isinstance(v, bool) for v in prox.values())
    finally:
        shutdown_registry()


def test_headless_runs_unaffected_when_dashboard_off():
    # the import + a plain snapshot must work with no dashboard started
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg, 0)
        d.takeoff(100)
        snap = DebugProbe(reg).snapshot()
        assert "drones" in snap and "sim_time" in snap
        # mode field exposed for the dashboard
        assert snap["drones"][0]["mode"] in ("idle", "blocking", "manual")
    finally:
        shutdown_registry()
