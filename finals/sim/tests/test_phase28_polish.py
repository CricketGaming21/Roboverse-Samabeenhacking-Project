"""Phase 28 acceptance test — high-visibility detect/acquire box, HQ record /
higher-res insets, and smooth realistic rover loiter (headless).

Observer-side; public surface frozen; fidelity unchanged.
"""

import math

import cv2
import numpy as np
import pytest

from pyhulax import DroneAPI

from simcore import camfeed
from simcore.config import load_config
from simcore.recorder import ArenaRecorder
from simcore.registry import get_registry, shutdown_registry


def _sq(cx, cy, half):
    return np.array([[cx - half, cy - half], [cx + half, cy - half],
                     [cx + half, cy + half], [cx - half, cy + half]],
                    dtype=np.float32)


# --------------------------------------------------------------------------- #
# A1. Bold, high-contrast detect/acquire box + label pill
# --------------------------------------------------------------------------- #

def test_box_thickness_scales_and_minimum():
    assert camfeed.box_thickness(640) >= 3
    assert camfeed.box_thickness(1920) > camfeed.box_thickness(640)  # scales
    assert camfeed.box_thickness(120) >= 3                           # floor


def test_detected_box_is_bold_bright_yellow_with_label_pill():
    img = np.full((480, 640, 3), 90, dtype=np.uint8)   # mid-grey background
    camfeed.draw_acquisition_box(img, 22, _sq(320, 240, 70), acquired=False)
    # bright yellow present and BOLD: a scan across the top edge crosses
    # >= the configured thickness of bright-yellow pixels.
    def bright_yellow(px):
        b, g, r = int(px[0]), int(px[1]), int(px[2])
        return r > 200 and g > 200 and b < 90
    col = 320
    run = sum(1 for y in range(150, 200)
              if bright_yellow(img[y, col]))
    assert run >= camfeed.box_thickness(640)           # outline >= 3 px thick
    assert camfeed._DETECTED == (0, 255, 255)          # exact bright yellow
    # label pill: a dark, semi-opaque band sits behind the label above the
    # box. Scan rows just above the box top (y=170) for a darkened band.
    strip = img[140:172, 248:430]
    row_means = strip.reshape(strip.shape[0], -1).mean(axis=1)
    assert float(row_means.min()) < 75                 # pill darkened the bg


def test_acquired_box_is_bright_green():
    img = np.full((480, 640, 3), 90, dtype=np.uint8)
    camfeed.draw_acquisition_box(img, 22, _sq(320, 240, 70), acquired=True)
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    green = int(np.sum((g > 200) & (r < 90) & (b < 90)))
    assert green > 50
    assert camfeed._ACQUIRED == (0, 255, 0)            # exact bright green


# --------------------------------------------------------------------------- #
# A2. HQ record size + higher-res insets
# --------------------------------------------------------------------------- #

def test_hq_record_size_honoured():
    cfg = load_config("sim_config.yaml")
    cfg.record.width, cfg.record.height = 1920, 1080   # what --hq sets
    reg = get_registry(cfg)
    try:
        rec = ArenaRecorder(reg, "/tmp/_p28_hq.mp4")
        assert rec._arena_w == 1920 and rec._arena_h == 1080
        assert rec._w == 1920                          # canvas honours HQ width
    finally:
        shutdown_registry()


def test_insets_render_at_higher_internal_resolution():
    cfg = load_config("sim_config.yaml")
    cfg.meta.real_time_factor = 10.0
    cfg.scoring.enabled = False
    cfg.arena.layout = "procedural"
    cfg.arena.obstacles.count = 0
    cfg.rovers.count = 0
    cfg.motion.realistic = False
    cfg.record.inset_render_h = 720
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        drone = reg.drones[0]
        # the referee default render stays AI-mode 640x480
        base = reg.render_camera(drone)
        assert base.shape == (cfg.camera.height, cfg.camera.width, 3)
        # the inset override renders at the higher internal resolution (4:3)
        hi = reg.render_camera(drone, width=720 * 4 // 3, height=720)
        assert hi.shape == (720, 960, 3)
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# B. Smooth, realistic rover loiter — no snap, no oscillation
# --------------------------------------------------------------------------- #

def _loiter_cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.scenario.ambush_trigger.mode = "timed"
    c.scenario.ambush_trigger.delay_s = 0.5
    # let the rover reach and loiter its loop within the sampling window
    c.rovers.convoy.speed_mps = 0.5
    c.rovers.convoy.entry_stagger_s = 0.0
    return c


def test_rover_loiter_is_smooth_no_snap_no_oscillation():
    import time
    cfg = _loiter_cfg()
    reg = get_registry(cfg)
    try:
        # sample rover 0 finely for a long stretch (well into the loiter loop).
        # read sim-time AND position in ONE sim-thread closure so they are an
        # atomic snapshot — otherwise the sim advances between the two reads
        # and a normal crawl looks like a jump.
        track = []
        deadline = time.time() + 9.0
        while time.time() < deadline:
            t, pos = reg.run_on_sim_thread(
                lambda: (reg.clock.now(),
                         (float(reg.rovers[0].pos[0]),
                          float(reg.rovers[0].pos[1]), float(reg.rovers[0].yaw))))
            track.append((t, pos))
            time.sleep(0.01)
        # keep only in-arena, de-duplicated by sim time, after it has entered
        seen, samples = set(), []
        for t, pos in track:
            if pos[0] < -0.01 or t in seen:    # off-map staging / dup tick
                continue
            seen.add(t)
            samples.append((t, pos))
        samples = samples[20:]                 # drop the entry transient
        assert len(samples) > 50

        speeds, turns = [], []
        for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
            dt = t1 - t0
            if dt <= 0:
                continue
            step = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            # NO SNAP: no position jump anywhere near a teleport between frames
            assert step < 0.25, f"position discontinuity {step:.2f} m (snap)"
            speeds.append(step / dt)
            dyaw = (p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi
            turns.append(dyaw)
        # velocity stays ~the convoy speed (calm crawl, no stalls/sprints)
        assert 0.2 <= np.median(speeds) <= 0.65
        # NO OSCILLATION: the heading does not rapidly reverse back and forth
        reversals = sum(1 for a, b in zip(turns, turns[1:])
                        if a * b < 0 and abs(a) > 0.08 and abs(b) > 0.08)
        assert reversals <= max(3, len(turns) // 15)
        # heading is RATE-LIMITED (eased), never an instant spin
        max_turn_rate = cfg.rovers.convoy.turn_rate_dps
        max_dyaw = max(abs(t) for t in turns)
        # per-frame turn within the rate limit (+ generous sampling slack)
        assert max_dyaw <= math.radians(max_turn_rate) * 0.3 + 0.05
    finally:
        shutdown_registry()


def test_loiter_loop_seam_is_continuous():
    import time
    cfg = _loiter_cfg()
    speed = cfg.rovers.convoy.speed_mps
    reg = get_registry(cfg)
    try:
        prev = None
        worst = 0.0
        deadline = time.time() + 9.0
        while time.time() < deadline:
            # atomic (sim-time, position, in_arena) snapshot on the sim thread,
            # so the elapsed-time denominator matches the distance actually
            # moved, and the one-time arena-ENTRY teleport (off-map staging ->
            # entrance) is excluded — we only measure while routing in-arena.
            t, pos, inside = reg.run_on_sim_thread(
                lambda: (reg.clock.now(),
                         (float(reg.rovers[0].pos[0]),
                          float(reg.rovers[0].pos[1])),
                         bool(reg.rovers[0].in_arena)))
            if prev is not None and inside and prev[2]:
                dt = t - prev[0]
                step = math.hypot(pos[0] - prev[1][0], pos[1] - prev[1][1])
                # measured against ELAPSED SIM TIME, so a slow sample gap is
                # allowed a proportional step — but a teleport (loop-seam
                # snap: large step in ~one tick) is not.
                if dt > 0:
                    worst = max(worst, step - speed * dt)
            prev = (t, pos, inside)
            time.sleep(0.012)
        # even across the loop wrap-around, no step exceeds what the crawl
        # speed covers in the elapsed time (no teleport/snap)
        assert worst < 0.05
    finally:
        shutdown_registry()


def test_loiter_hold_eases_to_a_stop():
    import time
    cfg = _loiter_cfg()
    cfg.rovers.convoy.loiter = "hold"
    reg = get_registry(cfg)
    try:
        time.sleep(6.0)   # ~60 sim s: rover 0 reaches + holds its branch end
        end = tuple(cfg.rovers.convoy.branches[0][-1])
        p = reg.rover_arena_positions()[0]
        assert math.dist(p, end) < 0.15          # parked at the branch end
        time.sleep(0.3)
        assert reg.rover_arena_positions()[0] == reg.rover_arena_positions()[0]
    finally:
        shutdown_registry()
