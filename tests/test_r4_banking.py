"""R4 — banking: scan-while-transit + bank-and-release + evidence capture.

The camera SEES rovers it never used to BANK (banking only happened during deliberate
locks). R4 makes banking continuous (a gate-passing read banks during transit AND patrol),
releases immediately after a bank (no dwelling on an already-banked rover), and produces the
judge deliverable (one annotated PNG per rover + a gallery). The ≥40 px + 5-frame + in-frame
gate is unchanged; R2 persistence still banks an initially-out-of-cone rover.
"""

import math

import cv2
import numpy as np
import pytest

from mission.config import load_config
from mission.mission.phase2_search import (ScanBanker, lock_and_tag,
                                           persist_and_read, phase2_search,
                                           vantage_patrol)
from mission.mission.worker import DroneWorker
from mission.perception.aruco import confirm_with_aruco
from mission.perception.detector import (ClassicalRoverDetector, Detection,
                                         frame_bgr)
from mission.perception.evidence import EvidenceWriter, annotate, caption_for
from mission.planner.projection import CameraIntrinsics
from mission.world.mission_state import MissionState
from mission.world.taskboard import TaskBoard
from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb
from tests.fakes.fake_pyhulax import CameraPitchMode, Marker

pytestmark = pytest.mark.r4

DICT = "DICT_6X6_250"
NOSLEEP = lambda _s: None
INTR = CameraIntrinsics(640, 480, 71.0)


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def _drone_over(world, n, e, *, alt_cm=110.0, pitch_down=90.0):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm = n, e, alt_cm
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, pitch_down)
    d.set_video_stream(True)
    s = d.create_video_stream()
    s.start()
    return d, s


def _world(rovers, *, crates=None, gimbal=False):
    w = fpx.FakeWorld(crates=crates or [])
    w.gimbal_enabled = gimbal
    w.pads = []
    w.rovers = list(rovers)
    return w


# --------------------------------------------------------------------------- #
# MissionState — drone_id + path provenance (R4)
# --------------------------------------------------------------------------- #
def test_mission_state_records_drone_id_and_path():
    st = MissionState()
    assert st.bank(20, None, (5.0, 3.0), 1.0, drone_id=2, path="logs/evidence/rover_20.png")
    e = st.evidence()[20]
    assert e.drone_id == 2 and e.path.endswith("rover_20.png")
    # de-dup by id — a second bank (even a different drone) does not overwrite
    assert st.bank(20, None, (9.0, 1.0), 2.0, drone_id=0) is False
    assert st.evidence()[20].drone_id == 2


def test_mission_state_set_path_after_dedup():
    st = MissionState()
    st.bank(45, None, (4.0, 2.0), 3.0, drone_id=1)
    st.set_path(45, "logs/evidence/rover_45.png")
    assert st.evidence()[45].path.endswith("rover_45.png")
    st.set_path(999, "ignored")          # unknown id → no-op, no raise


def test_mission_state_bank_backward_compatible():
    st = MissionState()                  # legacy 4-positional call still works (drone_id/path None)
    assert st.bank(22, None, (3.0, 3.0), 1.0)
    e = st.evidence()[22]
    assert e.drone_id is None and e.path is None


# --------------------------------------------------------------------------- #
# EvidenceWriter — annotate + PNG + gallery
# --------------------------------------------------------------------------- #
def test_caption_format():
    assert caption_for(1, 43.7, (4.40, 1.35)) == "drone 1 · t=43.7s · (4.40, 1.35) m"
    assert caption_for(None, 0.0, None) == "drone ? · t=0.0s · (n/a)"


def test_annotate_draws_box_without_mutating_source():
    src = np.full((80, 120, 3), 50, np.uint8)
    out = annotate(src, (10, 12, 30, 28), 67, "drone 1 · t=1.0s · (4.0, 2.0) m")
    assert out.shape == src.shape
    assert np.array_equal(src, np.full((80, 120, 3), 50, np.uint8))   # source untouched
    assert not np.array_equal(out, src)                               # something was drawn
    # the green detection box was drawn (a bright-green pixel exists where flat grey had none)
    green = (out[:, :, 1] > 180) & (out[:, :, 0] < 80) & (out[:, :, 2] < 80)
    assert green.any()


def test_evidence_writer_capture_and_gallery(tmp_path):
    ew = EvidenceWriter(tmp_path)
    frame = np.full((48, 64, 3), 120, np.uint8)
    annotated, path = ew.capture(frame, (8, 8, 20, 20), 67, drone_id=1, t=2.0,
                                 xy=(4.4, 1.35))
    assert path.endswith("rover_67.png")
    assert (tmp_path / "rover_67.png").exists()
    # PNG is a real image of the right size
    img = cv2.imread(path)
    assert img is not None and img.shape == (48, 64, 3)
    # gallery lists the banked id + caption
    st = MissionState()
    st.bank(67, annotated, (4.4, 1.35), 2.0, drone_id=1, path=path)
    idx = ew.gallery(st.evidence())
    html_text = (tmp_path / "index.html").read_text(encoding="utf-8")
    assert idx.endswith("index.html")
    assert "rover_67.png" in html_text
    assert "ID 67" in html_text
    assert "drone 1" in html_text and "(4.40, 1.35) m" in html_text


def test_gallery_lists_all_banked_ids(tmp_path):
    ew = EvidenceWriter(tmp_path)
    st = MissionState()
    for mid in (11, 45, 67):
        st.bank(mid, None, (4.0, 2.0), 1.0, drone_id=0, path=str(tmp_path / f"rover_{mid}.png"))
    ew.gallery(st.evidence())
    text = (tmp_path / "index.html").read_text(encoding="utf-8")
    for mid in (11, 45, 67):
        assert f"ID {mid}" in text and f"rover_{mid}.png" in text


# --------------------------------------------------------------------------- #
# ScanBanker — the continuous gate (≥40 px + in-frame + 5 CONSECUTIVE frames)
# --------------------------------------------------------------------------- #
def _banker(d, w, st, *, allow=frozenset({20}), evidence=None, in_zone=None,
            gimbal=90.0, search_pitch=52.0, taskboard=None):
    u = fuwb.FakeUWBParserThread(world=w)
    return ScanBanker(d, u, 0, st, allow=set(allow), intrinsics=INTR, gimbal_deg=gimbal,
                      search_pitch_deg=search_pitch, evidence=evidence,
                      taskboard=taskboard or TaskBoard(), in_zone=in_zone,
                      clock=lambda: w.clock)


def test_scan_banker_gate_px_and_in_frame():
    """The gate the referee scores on: ≥40 px AND fully in-frame (with a margin)."""
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)])
    d, _s = _drone_over(w, 5.0, 3.0)
    b = _banker(d, w, MissionState())
    assert b._gate((100, 100, 50, 50), (480, 640)) is True       # 50 px, well inside
    assert b._gate((100, 100, 30, 30), (480, 640)) is False      # 30 px < 40 → too small
    assert b._gate((2, 2, 50, 50), (480, 640)) is False          # inside the 8-px margin (edge)
    assert b._gate((600, 100, 50, 50), (480, 640)) is False      # runs off the right edge


def test_scan_banker_banks_after_5_consecutive_frames():
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)])
    d, s = _drone_over(w, 5.0, 3.0)                  # nadir over the rover, gimbal off
    st = MissionState()
    b = _banker(d, w, st)
    for _ in range(4):                               # 4 qualifying frames → NOT yet banked
        out = b.scan(frame_bgr(s.latest_frame))
        assert not out.banked and not st.is_tagged(20)
    out = b.scan(frame_bgr(s.latest_frame))          # the 5th consecutive → banked
    assert 20 in out.banked and st.is_tagged(20)
    fpx.set_active_world(None)


def test_scan_banker_streak_resets_on_gap():
    """A non-consecutive read restarts the 5-frame count (the gate is CONSECUTIVE frames)."""
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)])
    d, s = _drone_over(w, 5.0, 3.0)
    st = MissionState()
    b = _banker(d, w, st)
    for _ in range(4):                               # streak → 4
        b.scan(frame_bgr(s.latest_frame))
    d.n, d.e = 9.5, 5.5                               # marker leaves the FOV → streak breaks
    b.scan(frame_bgr(s.latest_frame))
    assert not st.is_tagged(20)
    d.n, d.e = 5.0, 3.0                               # back over the rover
    for _ in range(4):                               # only 4 again → still not banked
        out = b.scan(frame_bgr(s.latest_frame))
        assert not out.banked
    out = b.scan(frame_bgr(s.latest_frame))          # 5 consecutive after the gap → banked
    assert 20 in out.banked
    fpx.set_active_world(None)


def test_scan_banker_ignores_already_banked():
    """Bank-and-release: an already-banked id is ignored — no re-bank, no streak."""
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)])
    d, s = _drone_over(w, 5.0, 3.0)
    st = MissionState()
    st.bank(20, None, (5.0, 3.0), 0.0)               # already banked
    b = _banker(d, w, st)
    for _ in range(6):
        out = b.scan(frame_bgr(s.latest_frame))
        assert not out.banked
    assert st.count() == 1                           # never re-banked / double-counted
    assert 20 not in b._streak                        # not even tracked
    fpx.set_active_world(None)


def test_scan_banker_bank_writes_evidence_and_records_provenance(tmp_path):
    w = _world([Marker(67, 5.0, 3.0, size_m=0.20)])
    d, s = _drone_over(w, 5.0, 3.0)
    st = MissionState()
    ew = EvidenceWriter(tmp_path)
    b = _banker(d, w, st, allow={67}, evidence=ew)
    for _ in range(5):
        b.scan(frame_bgr(s.latest_frame))
    assert st.is_tagged(67)
    e = st.evidence()[67]
    assert e.drone_id == 0 and e.path.endswith("rover_67.png")
    assert (tmp_path / "rover_67.png").exists()
    assert "ID 67" in (tmp_path / "index.html").read_text(encoding="utf-8")
    fpx.set_active_world(None)


def test_scan_banker_zone_gate_blocks_out_of_zone_but_logs():
    """An in-view rover OUTSIDE the zone is logged to the taskboard but not banked."""
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)])
    d, s = _drone_over(w, 5.0, 3.0)
    st, tb = MissionState(), TaskBoard()
    b = _banker(d, w, st, taskboard=tb, in_zone=lambda xy: False)   # nothing is "in zone"
    for _ in range(6):
        out = b.scan(frame_bgr(s.latest_frame))
        assert not out.banked
    assert not st.is_tagged(20)
    assert tb.track_for(20) is not None              # the sighting was still logged
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# scan-while-transit — bank a gate-passing read while FLYING (no dwell-lock)
# --------------------------------------------------------------------------- #
def test_scan_while_transit_banks_during_hop():
    """The rover sits on the A->B transit path; the vantage is far past it and the dwell does
    NOT scan — so a bank can only have happened WHILE the drone was flying past."""
    w = _world([Marker(20, 4.0, 2.0, size_m=0.22)])
    d, s = _drone_over(w, 1.0, 2.0)                  # start south, nadir
    u = fuwb.FakeUWBParserThread(world=w)
    st = MissionState()
    b = ScanBanker(d, u, 0, st, allow={20}, intrinsics=INTR, gimbal_deg=90.0,
                   clock=lambda: w.clock)

    def on_frame():
        f = s.latest_frame
        if f is not None:
            b.scan(frame_bgr(f))
        return False

    # on_dwell=False (no dwell scanning); vantage 3 m north of the rover → not seen at the dwell
    vantage_patrol(d, u, 0, [{"xy": (7.0, 2.0), "gimbal_deg": 90, "dwell_s": 0.2}],
                   lambda: False, gimbal_deg=90.0, alt_m=1.1, rate_hz=20.0,
                   sleep=NOSLEEP, on_frame=on_frame)
    assert st.is_tagged(20)                          # banked mid-transit, no dwell-lock
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# bank-and-release — camera returns to search pitch; already-banked → no re-lock
# --------------------------------------------------------------------------- #
def test_bank_and_release_returns_camera_to_search_pitch():
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)])
    d, s = _drone_over(w, 5.0, 3.0, pitch_down=90.0)   # reading at nadir
    st = MissionState()
    b = _banker(d, w, st, gimbal=90.0, search_pitch=52.0)
    for _ in range(5):
        b.scan(frame_bgr(s.latest_frame))
    assert st.is_tagged(20)
    assert d.pitch_down_deg == pytest.approx(52.0)     # released → back to the search pitch
    fpx.set_active_world(None)


def test_already_banked_id_triggers_no_relock():
    """lock_and_tag on an already-banked id is a no-op: no servo, no camera move (no re-lock)."""
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)])
    d, s = _drone_over(w, 5.0, 3.0, pitch_down=52.0)
    st = MissionState()
    st.bank(20, None, (5.0, 3.0), 0.0)
    n0, p0 = d.manual_calls, d.pitch_down_deg
    ok = lock_and_tag(d, s, Detection((0, 0, 9, 9), 1.0, marker_id=20), st,
                      intrinsics=INTR, sleep=NOSLEEP)
    assert ok is False
    assert d.manual_calls == n0                       # no servo issued
    assert d.pitch_down_deg == pytest.approx(p0)      # camera untouched
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# phase2_search end-to-end — evidence per banked id + gallery, no double-count
# --------------------------------------------------------------------------- #
def test_phase2_banks_write_evidence_and_gallery(tmp_path):
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20), Marker(21, 5.0, 2.5, size_m=0.20)])
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.takeoff(110)
    u = fuwb.FakeUWBParserThread(world=w)
    s = d.create_video_stream(); d.set_video_stream(True); s.start()
    st, tb = MissionState(), TaskBoard()
    ew = EvidenceWriter(tmp_path)
    phase2_search(d, u, 0, [{"xy": (5.0, 3.0), "gimbal_deg": 90, "dwell_s": 1.0}],
                  s, st, tb, bubble=None, all_ids=[20, 21], rover_ids=[20, 21],
                  dictionary=DICT, budget_cycles=1, mopup_extra_cycles=0,
                  intrinsics=INTR, evidence=ew, sleep=NOSLEEP)
    assert st.tagged() == {20, 21} and st.count() == 2          # both banked, no double-count
    assert (tmp_path / "rover_20.png").exists()
    assert (tmp_path / "rover_21.png").exists()
    text = (tmp_path / "index.html").read_text(encoding="utf-8")
    assert "ID 20" in text and "ID 21" in text                 # gallery lists every banked id
    assert st.evidence()[20].drone_id == 0
    assert st.evidence()[20].path.endswith("rover_20.png")
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# R2 persistence STILL banks an initially-out-of-cone rover (now with evidence)
# --------------------------------------------------------------------------- #
def test_persist_with_banker_banks_out_of_cone_and_writes_evidence(tmp_path):
    w = _world([Marker(45, 4.0, 2.0, size_m=0.20, gimbal_phase_deg=0.0)], gimbal=True)
    d, s = _drone_over(w, 3.0, 2.0, pitch_down=52.0)
    assert w.marker_readable(w.rovers[0], (d.n, d.e)) is False   # out of cone at t=0
    u = fuwb.FakeUWBParserThread(world=w)
    st = MissionState()
    ew = EvidenceWriter(tmp_path)
    b = ScanBanker(d, u, 0, st, allow={45}, intrinsics=INTR, gimbal_deg=52.0,
                   search_pitch_deg=52.0, evidence=ew, clock=lambda: w.clock)
    mid = persist_and_read(
        d, s, (4.0, 2.0), st, allow={45}, dictionary=DICT, gimbal_deg=52.0,
        intrinsics=INTR, uwb=u, tag_id=0, alt_m=1.1, presence=ClassicalRoverDetector(),
        footprints=(), bounds=None, persist_timeout_s=9.0, rate_hz=20.0, sleep=NOSLEEP,
        clock=lambda: w.clock, banker=b)
    assert mid == 45 and st.is_tagged(45)             # gimbal swept in → banked while persisting
    assert (tmp_path / "rover_45.png").exists()        # evidence written for the persisted bank
    assert st.evidence()[45].drone_id == 0
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# R2-persistence not regressed: an ALREADY-BANKED marker in frame must not block
# persisting on a DIFFERENT out-of-cone rover (the convoy-id-67 stall)
# --------------------------------------------------------------------------- #
def test_nearest_presence_excludes_decoded_marker_blob():
    """A decoded (explained) marker's blob is skipped by the presence channel, so the persist
    hold targets the genuinely-unread out-of-cone body, not an already-handled marker."""
    from mission.mission.phase2_search import _nearest_presence
    w = _world([Marker(45, 4.0, 2.0, size_m=0.20)])    # gimbal off → 45 decodes + renders a blob
    d, s = _drone_over(w, 4.0, 2.0)                     # nadir over 45
    bgr = frame_bgr(s.latest_frame)
    bboxes = [x.bbox for x in confirm_with_aruco(bgr, DICT) if x.marker_id == 45]
    assert bboxes                                       # 45 is decoded
    u = fuwb.FakeUWBParserThread(world=w)
    det = ClassicalRoverDetector()
    # without exclusion, 45's marker blob IS a presence candidate
    assert _nearest_presence(det, bgr, None, d, 90.0, INTR, u, 0, (), None, 100) is not None
    # excluding 45's decoded bbox removes the only blob → nothing to persist on
    assert _nearest_presence(det, bgr, None, d, 90.0, INTR, u, 0, (), None, 100,
                             exclude_bboxes=bboxes) is None
    fpx.set_active_world(None)


def test_persist_not_blocked_by_banked_marker_in_frame(tmp_path):
    """The R2 regression guard: a drone seeing an ALREADY-BANKED rover's decodable marker AND a
    DIFFERENT out-of-cone rover's body in the same frame must still PERSIST on the body and bank
    it once the gimbal sweeps the marker into the cone (id 67 went unbanked when this broke)."""
    w = _world([Marker(67, 4.0, 2.0, size_m=0.20, gimbal_phase_deg=0.0),     # out of cone @ t=0
                Marker(11, 4.0, 1.5, size_m=0.20, gimbal_phase_deg=153.0)],  # readable @ t=0
               gimbal=True)
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e, d.alt_cm = 3.0, 2.0, 110.0
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 52.0)
    s = d.create_video_stream(); d.set_video_stream(True); s.start()
    u = fuwb.FakeUWBParserThread(world=w)
    st, tb = MissionState(), TaskBoard()
    st.bank(11, None, (4.0, 1.5), 0.0)                 # 11 already banked (decodable, in frame)
    assert w.marker_readable(w.rovers[0], (3.0, 2.0)) is False   # 67 out of cone at t=0
    phase2_search(d, u, 0, [{"xy": (3.0, 2.0), "gimbal_deg": 52, "dwell_s": 0.5}],
                  s, st, tb, bubble=None, all_ids=[11, 67], rover_ids=[11, 67],
                  dictionary=DICT, budget_cycles=1, mopup_extra_cycles=0, gimbal_deg=52.0,
                  intrinsics=INTR, presence=ClassicalRoverDetector(),
                  persist_timeout_s=9.0, evidence=EvidenceWriter(tmp_path),
                  sleep=NOSLEEP, clock=lambda: w.clock)
    assert st.is_tagged(67)                            # persisted past the banked-11 marker → banked
    assert (tmp_path / "rover_67.png").exists()
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# Bounded persistence — a permanently-unreadable rover never hangs the run
# --------------------------------------------------------------------------- #
def test_permanently_unreadable_rover_does_not_hang():
    """A rover whose marker is OUT OF READ RANGE (tiny / never decodes even when geometrically
    in-cone) must NOT hang Phase-2: persist times out, the light-orbit fallback fires, the rover
    is RELEASED, and the run terminates (here via the Phase-2 wall-clock cap). The rover is never
    banked. (`--timeout=60` would also catch a true hang; this asserts the bound explicitly.)"""
    w = _world([Marker(20, 4.0, 2.0, size_m=0.04, gimbal_phase_deg=0.0)], gimbal=True)  # too small
    fpx.set_active_world(w)                                                              # to decode
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e, d.alt_cm = 3.0, 2.0, 110.0               # offset → marker periodically in-cone…
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 52.0)
    s = d.create_video_stream(); d.set_video_stream(True); s.start()
    u = fuwb.FakeUWBParserThread(world=w)
    st, tb = MissionState(), TaskBoard()
    phases = []
    banked = phase2_search(
        d, u, 0, [{"xy": (3.0, 2.0), "gimbal_deg": 52, "dwell_s": 0.5}], s, st, tb,
        bubble=None, all_ids=[20], rover_ids=[20], dictionary=DICT, gimbal_deg=52.0,
        intrinsics=INTR, presence=ClassicalRoverDetector(),
        persist_timeout_s=0.8, max_orbits=1, orbit_step_m=0.5,
        budget_cycles=50, dwell_s=0.5,                 # a huge cycle budget → ONLY the cap can stop it
        phase_budget_s=8.0, now=lambda: w.clock,       # hard wall-clock cap (sim-seconds here)
        sleep=NOSLEEP, clock=lambda: w.clock,
        on_step=lambda i: phases.append(i.get("phase")))
    assert banked == set() and not st.is_tagged(20)    # never banked (out of read range)
    assert "orbit" in phases                           # the light-orbit fallback fired
    assert w.clock <= 8.0 + 4.0                         # terminated at the cap (+ margin), no runaway
    fpx.set_active_world(None)


def test_phase2_terminates_when_marker_never_reads_even_overhead():
    """Belt-and-braces: even with NO orbit budget, an unreadable rover times out and the run ends
    within the wall-clock cap — the search always returns so the mission can land all in finally."""
    w = _world([Marker(20, 4.0, 2.0, size_m=0.04)], gimbal=True)
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e, d.alt_cm = 4.0, 2.0, 110.0               # overhead → marker NEVER readable
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 52.0)
    s = d.create_video_stream(); d.set_video_stream(True); s.start()
    u = fuwb.FakeUWBParserThread(world=w)
    st, tb = MissionState(), TaskBoard()
    banked = phase2_search(
        d, u, 0, [{"xy": (4.0, 2.0), "gimbal_deg": 52, "dwell_s": 0.5}], s, st, tb,
        bubble=None, all_ids=[20], rover_ids=[20], dictionary=DICT, gimbal_deg=52.0,
        intrinsics=INTR, presence=ClassicalRoverDetector(), persist_timeout_s=0.5,
        max_orbits=0, budget_cycles=50, phase_budget_s=5.0, now=lambda: w.clock,
        sleep=NOSLEEP, clock=lambda: w.clock)
    assert banked == set() and not st.is_tagged(20)    # never banked, never hung
    assert w.clock <= 5.0 + 3.0                         # stopped at the cap
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# Phase 1 is untouched — R4 is Phase-2-only, no banking during landing
# --------------------------------------------------------------------------- #
def test_phase1_does_not_bank(tmp_path):
    """Even with a rover marker rendered in the arena, a Phase-1 land banks NOTHING (the camera
    is off in Phase 1 — R1 — and R4 added no banking to the landing path)."""
    cfg = load_config()
    w = _world([Marker(20, 5.0, 3.0, size_m=0.20)], crates=[])
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    u = fuwb.FakeUWBParserThread(world=w)
    s = d.create_video_stream()
    st = MissionState()
    worker = DroneWorker(d, u, 0, cfg, footprints=[], stream=s, sleep=NOSLEEP)
    worker.run_phase1((0.6, 1.0))                      # land on a clear pad
    assert worker.landed_ok is True
    assert st.count() == 0                             # Phase 1 banked nothing
    assert not (tmp_path / "index.html").exists()      # no evidence artifact produced
    fpx.set_active_world(None)
