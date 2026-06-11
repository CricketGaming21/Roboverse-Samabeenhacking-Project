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

from mission.perception.aruco import confirm_with_aruco
from mission.perception.detector import frame_bgr
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
    html_text = (tmp_path / "index.html").read_text()
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
    text = (tmp_path / "index.html").read_text()
    for mid in (11, 45, 67):
        assert f"ID {mid}" in text and f"rover_{mid}.png" in text
