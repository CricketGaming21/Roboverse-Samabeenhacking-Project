"""Phase 7 acceptance test — referee scoring, thrash monitor, top-down view,
full 3-drone smoke run (headless).

Gate NEGATIVES (too small / clipped / not held) drive the referee's judge
directly with synthetic detections — deterministic, no frame-timing races.
The POSITIVE path (held + centred + large enough => banked once, deduped
across drones) runs end-to-end through real rendered frames.
"""

import time

import numpy as np
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode, Direction

from scripts.smoke_test import run_smoke
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry
from simcore.scoring import Referee
from simcore.viz import TopDownView



@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.motion.realistic = False  # crisp snap motion: exact geometry under test
    c.arena.layout = "procedural"  # authored map off: clean nadir views
    c.arena.obstacles.count = 0
    c.scenario.phases = "ambush"   # part-2 referee judges only in AMBUSH
    c.rovers.motion = "patrol"
    c.rovers.patrol.speed_mps = 0.0  # parked rover targets at their spawns
    return c


@pytest.fixture()
def sim(cfg):
    reg = get_registry(cfg)
    yield reg
    shutdown_registry()


def _connect(cfg, index=0) -> DroneAPI:
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


def _hover_over(d, cfg, index, north, east, z_cm=150):
    """Park drone <index>'s camera over an arena point, pitched down."""
    start = cfg.drones.units[index].start
    d.takeoff(150)
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.move_to((east - start[1]) * 100.0,
              (north - start[0]) * 100.0 - cfg.camera.mount_offset_m * 100.0,
              z_cm)


def _wait_banked(reg, marker_id, timeout_s=8.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if marker_id in reg.referee.banked_ids():
            return True
        time.sleep(0.02)
    return False


def _quad(cx, cy, side):
    """Synthetic axis-aligned marker quad, pixel coords."""
    h = side / 2.0
    return np.array([[cx - h, cy - h], [cx + h, cy - h],
                     [cx + h, cy + h], [cx - h, cy + h]], dtype=np.float32)


# --------------------------------------------------------------------------- #
# The gate (white-box: synthetic detections through the real judge)
# --------------------------------------------------------------------------- #

def test_gate_negatives_do_not_score(sim, cfg):
    ref = Referee(sim)  # not started: judged manually, no render thread
    n = cfg.scoring.hold_frames
    big, margin = cfg.scoring.min_marker_px + 20, cfg.scoring.frame_margin_px

    # Too small: one px under the size gate, held forever -> never banks.
    for _ in range(3 * n):
        ref._judge(0, [(50, _quad(320, 240, cfg.scoring.min_marker_px - 1))])
    assert ref.score() == 0

    # Clipped: big enough but a corner inside the frame margin -> never banks.
    for _ in range(3 * n):
        ref._judge(0, [(51, _quad(margin + 10, 240, big))])  # left edge
    assert ref.score() == 0

    # Glimpse: valid but the consecutive hold keeps breaking -> never banks.
    for _ in range(4):
        for _ in range(n - 1):
            ref._judge(0, [(52, _quad(320, 240, big))])
        ref._judge(0, [])  # one missed frame resets the hold
    assert ref.score() == 0


def test_gate_positive_and_cross_drone_dedup(sim, cfg):
    ref = Referee(sim)
    n = cfg.scoring.hold_frames
    big = cfg.scoring.min_marker_px + 20

    banked = []
    for _ in range(n):
        banked += ref._judge(0, [(60, _quad(320, 240, big))])
    assert banked == [60]            # banked exactly on the nth held frame
    assert ref.score() == 1

    for _ in range(3 * n):           # drone 1 stares at the same id
        assert ref._judge(1, [(60, _quad(320, 240, big))]) == []
    assert ref.score() == 1          # never double-counted
    assert ref.banked()[0].drone_index == 0  # first holder keeps attribution

    for _ in range(n):               # but drone 1 CAN bank a new id
        ref._judge(1, [(61, _quad(320, 240, big))])
    assert ref.banked_ids() == {60, 61}


def test_explicit_capture_mode(sim, cfg):
    cfg.scoring.mode = "explicit"
    ref = Referee(sim)
    big = cfg.scoring.min_marker_px + 20
    for _ in range(4 * cfg.scoring.hold_frames):
        ref._judge(0, [(70, _quad(320, 240, big))])
    assert ref.score() == 0          # explicit mode: no auto banking
    assert ref.register_capture(0) == [70]   # internal hook banks it
    assert ref.register_capture(0) == []     # idempotent
    assert ref.score() == 1
    cfg.scoring.mode = "auto"


# --------------------------------------------------------------------------- #
# End-to-end: rendered frames -> referee bank, deduped across drones
# --------------------------------------------------------------------------- #

def test_held_centred_read_banks_once_across_drones(sim, cfg):
    # Part-2 targets are the ROVER markers (parked at their spawns here).
    (r0n, r0e), (r1n, r1e) = sim.rover_arena_positions()[:2]
    id0, id1 = sim.rovers[0].marker_id, sim.rovers[1].marker_id

    def _record(marker_id):
        return next((b for b in sim.referee.banked()
                     if b.marker_id == marker_id), None)

    d0 = _connect(cfg, 0)
    _hover_over(d0, cfg, 0, north=r0n, east=r0e)   # over rover 0
    assert _wait_banked(sim, id0), "held centred read did not bank"
    record = _record(id0)
    assert record.drone_index == 0
    assert record.sim_time > 0

    d1 = _connect(cfg, 1)                          # second drone, same rover
    _hover_over(d1, cfg, 1, north=r0n, east=r0e)
    d1.hover(1.0)                                  # plenty of held frames
    records = [b for b in sim.referee.banked() if b.marker_id == id0]
    assert len(records) == 1                       # banked exactly once
    assert records[0].drone_index == 0             # attribution kept

    assert _record(id1) is None                    # fresh target so far
    start1 = cfg.drones.units[1].start
    d1.move_to((r1e - start1[1]) * 100.0,
               (r1n - start1[0]) * 100.0 - 10.0, 150)
    assert _wait_banked(sim, id1), "drone 1 failed to bank a fresh id"
    assert _record(id1).drone_index == 1


# --------------------------------------------------------------------------- #
# Thrash monitor
# --------------------------------------------------------------------------- #

def test_thrash_warnings_fire_on_rapid_recommand(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    base = sim.monitor.snapshot()
    assert base["rate_warnings"].get(0, 0) == 0
    d.move(Direction.FORWARD, 100, blocking=False)   # rapid-fire re-commands
    d.move(Direction.BACK, 50, blocking=False)
    d.move(Direction.LEFT, 50, blocking=False)
    snap = sim.monitor.snapshot()
    assert snap["rate_warnings"].get(0, 0) >= 2      # << min interval apart
    assert snap["preemptions"].get(0, 0) >= 2        # each killed the last
    report = sim.monitor.format_report()
    assert "drone 0" in report and "rate warnings" in report


def test_normal_paced_commands_do_not_warn(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)        # blocking commands complete before the next starts
    d.move(Direction.FORWARD, 100)
    d.rotate(90)
    d.land()
    snap = sim.monitor.snapshot()
    assert snap["rate_warnings"].get(0, 0) == 0
    assert snap["preemptions"].get(0, 0) == 0
    assert snap["commands"].get(0, 0) == 4


# --------------------------------------------------------------------------- #
# Top-down view (headless) + the full smoke run
# --------------------------------------------------------------------------- #

def test_topdown_renders_png_headless(sim, cfg, tmp_path):
    d = _connect(cfg)
    d.takeoff(100)
    d.move(Direction.FORWARD, 100)
    view = TopDownView(sim)
    view.sample()
    out = tmp_path / "topdown.png"
    view.render_png(str(out))
    assert out.is_file() and out.stat().st_size > 10_000  # a real plot

    import cv2
    img = cv2.imread(str(out))
    assert img is not None and img.shape[0] > 200 and img.shape[1] > 200


def test_full_smoke_run_scores_and_reports_thrash():
    c = load_config("sim_config.yaml")   # FULL world: authored map + convoy
    c.meta.real_time_factor = 10.0
    result = run_smoke(c)
    # Part 1: all three drones land on their designated pads within tolerance
    assert result["landing_score"] == 3
    pads = sorted(p for _d, p, _e, _t in result["landings"])
    designated = sorted(p.id for p in c.pads if p.valid and p.designated)
    assert pads == designated
    assert all(err <= c.scoring.landing.tolerance_m
               for _d, _p, err, _t in result["landings"])
    # Part 2: only ROVER markers are banked (pads carry no marker)
    assert set(b[0] for b in result["banked"]) <= set(c.rovers.marker_ids)
    assert sum(result["monitor"]["preemptions"].values()) >= 1  # thrash demo
    assert sum(result["monitor"]["rate_warnings"].values()) >= 1
    assert result["sim_time"] > 0
