"""Phase 22 acceptance test — offscreen 3D arena recording (headless).

The recorder is a PASSIVE OBSERVER inside the one real run: it renders extra
offscreen frames via the guarded EGL path (never p.GUI) and writes an MP4.
It must not perturb the sim (same seed => same scoreboard), must produce
real (non-blank) frames, and must be fully gated by the flag.
"""

import copy

import cv2
import numpy as np
import pytest

from scripts.scenario_demo import run_scenario_demo
from simcore.config import load_config
from simcore.recorder import ArenaRecorder
from simcore.registry import get_registry, shutdown_registry


def _cfg(rtf=10.0):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = rtf
    # a compact but real two-phase episode (semantics unchanged)
    c.scenario.episode_seconds = 12.0
    c.scenario.ambush_seconds = 9.0
    c.scenario.deploy_timeout_s = 8.0
    c.scenario.ambush_trigger.mode = "on_all_landed"
    c.scenario.ambush_trigger.delay_s = 1.0
    return c


# --------------------------------------------------------------------------- #
# The offscreen render: real arena frame, never GUI
# --------------------------------------------------------------------------- #

def test_render_arena_is_real_offscreen_frame():
    cfg = _cfg()
    cfg.scoring.enabled = False
    reg = get_registry(cfg)
    try:
        import pybullet as p
        # the run is genuinely headless DIRECT — no GUI client anywhere
        assert reg.gui is False
        mode = reg.run_on_sim_thread(
            lambda: p.getConnectionInfo(reg.client)["connectionMethod"])
        assert mode == p.DIRECT
        rgb = reg.render_arena(320, 240)
        assert rgb is not None and rgb.shape == (240, 320, 3)
        assert rgb.dtype == np.uint8
        assert float(rgb.std()) > 5.0          # non-blank: real geometry
    finally:
        shutdown_registry()


def test_capture_frame_has_overlay_and_variance(tmp_path):
    cfg = _cfg()
    cfg.scoring.enabled = False
    cfg.record.width, cfg.record.height = 480, 320
    cfg.record.show_camera_insets = False  # bare arena frame under test here
    reg = get_registry(cfg)
    try:
        rec = ArenaRecorder(reg, str(tmp_path / "x.mp4"))
        bgr = rec.capture_frame()
        assert bgr.shape == (320, 480, 3)
        assert float(bgr.std()) > 5.0          # arena geometry present
        # the info overlay occupies the top banner (a filled black bar with
        # white text) — its rows differ from the un-overlaid render below
        assert bgr[0:86].std() > 0
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# A real recording: ~episode_seconds * fps frames, file written
# --------------------------------------------------------------------------- #

def test_record_run_writes_video_with_expected_frames(tmp_path):
    cfg = _cfg()
    cfg.record.fps = 20
    cfg.record.width, cfg.record.height = 480, 320
    cfg.record.show_camera_insets = False  # arena-frame-rate path (Phase 22);
    # insets quadruple the per-frame render cost (Phase 23 has its own checks)
    out = tmp_path / "run.mp4"
    result = run_scenario_demo(cfg, record_path=str(out))

    expected = cfg.scenario.episode_seconds * cfg.record.fps  # ~240
    assert result["record_frames"] >= 0.6 * expected
    assert result["record_frames"] <= 1.3 * expected
    # an mp4 file (or the PNG-sequence fallback dir) was produced, non-trivial
    if out.exists():
        assert out.stat().st_size > 10_000
        cap = cv2.VideoCapture(str(out))
        ok, frame = cap.read()
        cap.release()
        assert ok and frame is not None and float(frame.std()) > 5.0
    else:
        pngs = list((tmp_path / "run_frames").glob("*.png"))
        assert len(pngs) == result["record_frames"]


# --------------------------------------------------------------------------- #
# Fidelity: the recorder does not perturb the sim
# --------------------------------------------------------------------------- #

def test_recorded_scoreboard_equals_plain_run_same_seed(tmp_path):
    plain = run_scenario_demo(_cfg())
    rec = run_scenario_demo(_cfg(), record_path=str(tmp_path / "r.mp4"))
    # same seed => same run => identical scoring, recorder or not
    assert rec["landing_score"] == plain["landing_score"]
    assert rec["snapshot_score"] == plain["snapshot_score"]
    assert sorted(b[0] for b in rec["banked"]) == \
        sorted(b[0] for b in plain["banked"])
    assert rec["final_phase"] == plain["final_phase"] == "done"
    assert rec["record_frames"] > 0


# --------------------------------------------------------------------------- #
# Gating: no --record => no recorder, no overhead
# --------------------------------------------------------------------------- #

def test_no_record_path_means_no_capture():
    result = run_scenario_demo(_cfg())          # no record_path
    assert result.get("record_frames", 0) == 0


def test_record_path_never_connects_gui(monkeypatch, tmp_path):
    """Hard guard: recording must never open a p.GUI client."""
    import pybullet as p
    real_connect = p.connect
    seen = []

    def _spy_connect(mode, *a, **k):
        seen.append(mode)
        return real_connect(mode, *a, **k)

    monkeypatch.setattr(p, "connect", _spy_connect)
    cfg = _cfg()
    cfg.record.width, cfg.record.height = 320, 240
    run_scenario_demo(cfg, record_path=str(tmp_path / "g.mp4"))
    assert p.GUI not in seen                     # never connected GUI
    assert p.DIRECT in seen                       # headless DIRECT only
