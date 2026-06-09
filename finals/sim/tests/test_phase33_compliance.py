"""Phase 33 acceptance test — no-fly-over-obstacle + altitude-cap compliance
flagging (headless). A FLAGGED event, NOT a crash.

The flag lives in the DebugProbe snapshot (so --record + --dashboard can show
a per-drone VIOLATION), and the ComplianceMonitor logs onset/clear transitions
to logs/compliance. Public API surface unchanged.
"""

import math
import time

import pytest

from pyhulax import DroneAPI

from simcore import compliance, frames
from simcore.config import load_config
from simcore.debug import DebugProbe
from simcore.recorder import ArenaRecorder
from simcore.registry import get_registry, shutdown_registry

# A short ground crate from the authored map (height 0.4 m) — a drone at the
# recommended ~1.1 m flies OVER it (no collision) so we can flag the no-fly rule.
SHORT_CRATE_NE = (5.0, 3.0)
CLEAR_NE = (1.0, 1.0)          # crate-free lane (crates start ~north 3 m)


def _cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 12.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    return c


def _fly_to(d, reg, cfg, n, e, z_cm):
    fr = reg.run_on_sim_thread(lambda: reg.drones[0].takeoff_frame)
    x, y, _ = frames.arena_to_takeoff_cm(cfg, fr, n, e)
    d.move_to(x, y, z_cm)


def _comp(reg):
    return DebugProbe(reg).snapshot(referee_view=False)["drones"][0]["compliance"]


# --------------------------------------------------------------------------- #
# Flagging: over a crate, over the altitude cap, and the clean case
# --------------------------------------------------------------------------- #

def test_airborne_over_crate_is_flagged_not_a_crash():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(110)
        _fly_to(d, reg, cfg, *SHORT_CRATE_NE, 110)
        c = _comp(reg)
        assert c["over_obstacle"] is True
        assert c["violation"] is True and "over_crate" in c["reasons"]
        # NOT a crash: the drone is still flying and controllable afterwards
        assert reg.run_on_sim_thread(lambda: reg.drones[0].flying) is True
        assert d.get_battery() >= 0
    finally:
        shutdown_registry()


def test_altitude_cap_is_flagged():
    cfg = _cfg()
    cap = cfg.compliance.max_altitude_m
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(110)
        _fly_to(d, reg, cfg, *CLEAR_NE, int((cap + 0.5) * 100))  # above the cap
        c = _comp(reg)
        assert c["altitude_m"] > cap
        assert c["over_altitude"] is True
        assert c["violation"] is True and "altitude_cap" in c["reasons"]
    finally:
        shutdown_registry()


def test_recommended_height_over_clear_floor_not_flagged():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(110)
        _fly_to(d, reg, cfg, *CLEAR_NE, 110)            # ~1.1 m, clear floor
        c = _comp(reg)
        assert c["airborne"] is True
        assert c["over_obstacle"] is False and c["over_altitude"] is False
        assert c["violation"] is False
    finally:
        shutdown_registry()


def test_grounded_drone_is_not_flagged_even_over_a_crate():
    # the rule is for AIRBORNE drones — a grounded one never flags
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        drone = reg.drones[0]

        def park_over_crate():
            wx, wy, _ = frames.arena_to_world(cfg, *SHORT_CRATE_NE, 0.0)
            drone.pos[0], drone.pos[1] = wx, wy   # horizontally over the crate
            drone.flying = False                  # but on the ground

        reg.run_on_sim_thread(park_over_crate)
        c = compliance.check(cfg, drone, reg.layout.obstacles)
        assert c["airborne"] is False and c["violation"] is False
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Surfaced: in the snapshot, the cockpit MP4, and the compliance log
# --------------------------------------------------------------------------- #

def test_compliance_flag_present_in_debug_snapshot():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(110)
        snap = DebugProbe(reg).snapshot(referee_view=False)
        c = snap["drones"][0]["compliance"]
        # the renderers (record/dashboard) read exactly these fields
        for k in ("enabled", "airborne", "altitude_m", "over_obstacle",
                  "over_altitude", "violation", "reasons"):
            assert k in c
    finally:
        shutdown_registry()


def test_recorder_renders_violation_indicator():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        rec = ArenaRecorder(reg, "/tmp/_p33.mp4")
        viol = {"index": 0, "flying": True, "battery_pct": 90.0, "uwb_ok": True,
                "speed_mps": 0.1, "course_deg": None, "camera_pitch_deg": 0,
                "mode": "blocking", "goal": {"kind": "move_to"},
                "true": {"arena_ne_m": [5.0, 3.0]},
                "estimate": {"takeoff_cm": [10, 20], "drift_error_m": 0.01},
                "orientation_deg": {"yaw": 0, "pitch": 0, "roll": 0},
                "compliance": {"violation": True, "over_obstacle": True,
                               "over_altitude": False}}
        rows = rec._telemetry_lines(viol)
        texts = [t for t, _c in rows]
        assert any("VIOLATION" in t and "OVER CRATE" in t for t in texts)
        # no violation -> no VIOLATION row
        viol["compliance"] = {"violation": False, "over_obstacle": False,
                              "over_altitude": False}
        assert not any("VIOLATION" in t for t, _c in rec._telemetry_lines(viol))
    finally:
        shutdown_registry()


def test_compliance_monitor_logs_a_violation_record(tmp_path):
    cfg = _cfg()
    cfg.compliance.log_dir = str(tmp_path / "compliance")
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(110)
        _fly_to(d, reg, cfg, *SHORT_CRATE_NE, 110)   # hover over a crate
        logfile = tmp_path / "compliance" / "compliance.log"
        deadline = time.time() + 4.0
        while time.time() < deadline and not logfile.exists():
            time.sleep(0.05)
        assert logfile.exists(), "compliance monitor wrote no logs/compliance record"
        text = logfile.read_text()
        assert "VIOLATION" in text and "over_crate" in text
    finally:
        shutdown_registry()


def test_compliance_can_be_disabled():
    cfg = _cfg()
    cfg.compliance.enabled = False
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(110)
        _fly_to(d, reg, cfg, *SHORT_CRATE_NE, 110)
        c = _comp(reg)
        assert c["enabled"] is False and c["violation"] is False  # gated off
    finally:
        shutdown_registry()
