"""Video-stream warm-up: real H.264 negotiation can delay the first frame several seconds
after set_video_stream(True). wait_for_first_frame() blocks (bounded) for it; camera_check and
the Phase-2 worker use it so warm-up is never mistaken for "no rovers seen"."""

import pytest

from mission.config import load_config
from mission.control.avoidance import ReactiveGuard
from mission.mission.worker import DroneWorker
from mission.perception.video import wait_for_first_frame
from mission.planner.projection import CameraIntrinsics
from mission.world.mission_state import MissionState
from mission.world.taskboard import TaskBoard
from scripts import camera_check
from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb
from tests.fakes.fake_pyhulax import CameraPitchMode, Marker

NOSLEEP = lambda _s: None
INTR = CameraIntrinsics(640, 480, 71.0)


def _silent(*_a, **_k):
    pass


class _WarmingStream:
    """Yields None for the first `cold` polls (H.264 negotiating), then a frame object."""

    def __init__(self, cold: int):
        self._cold = cold
        self.polls = 0
        self.started = False

    def start(self):
        self.started = True

    @property
    def latest_frame(self):
        self.polls += 1
        return None if self.polls <= self._cold else object()


class _ColdStartStream:
    """Wraps a real FakeVideoStream but returns None for the first `cold` reads (warm-up)."""

    def __init__(self, inner, cold: int):
        self._inner = inner
        self._cold = cold
        self.reads = 0

    def start(self):
        self._inner.start()

    def stop(self):
        self._inner.stop()

    @property
    def latest_frame(self):
        self.reads += 1
        return None if self.reads <= self._cold else self._inner.latest_frame


# --------------------------------------------------------------------------- #
# wait_for_first_frame — bounded poller
# --------------------------------------------------------------------------- #
def test_waits_through_warmup_then_returns_frame():
    s = _WarmingStream(cold=20)                      # ~2 s of negotiation at 0.1 s polls
    frame = wait_for_first_frame(s, timeout_s=15.0, poll_s=0.1, sleep=NOSLEEP)
    assert frame is not None
    assert s.polls == 21                              # returned on the first non-None poll


def test_returns_none_on_timeout_and_is_bounded():
    sleeps = []
    s = _WarmingStream(cold=10**9)                    # never warms up
    frame = wait_for_first_frame(s, timeout_s=1.0, poll_s=0.1, sleep=sleeps.append)
    assert frame is None
    assert len(sleeps) <= 11                          # BOUNDED ≈ timeout_s/poll_s — never unbounded


def test_returns_immediately_when_already_warm():
    s = _WarmingStream(cold=0)
    frame = wait_for_first_frame(s, sleep=NOSLEEP)
    assert frame is not None and s.polls == 1         # no wasted polls / sleeps


# --------------------------------------------------------------------------- #
# camera_check waits through warm-up instead of giving up in 3 s
# --------------------------------------------------------------------------- #
def test_camera_check_waits_through_warmup_and_decodes():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(23, 5.0, 3.0, size_m=0.20)]
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e, d.alt_cm = 5.0, 3.0, 110.0
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    # 25 cold reads >> the 3-frame decode loop: without the warm-up wait the loop would see
    # only None and miss the marker.
    cold = _ColdStartStream(d.create_video_stream(), cold=25)
    seen = camera_check.scan_frames(d, cold, frames=3, warmup_timeout_s=15.0,
                                    sleep=NOSLEEP, log=_silent)
    assert 23 in {mid for mid, _px in seen}
    assert d.manual_calls == 0 and d.alt_cm == 110.0  # still READ-ONLY
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# worker Phase-2 waits for warm-up before detection (cold start tags as well as warm)
# --------------------------------------------------------------------------- #
def _single_rover_world():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(20, 2.0, 1.0, size_m=0.2)]
    w.pads = []
    w.drone_starts = {0: (2.0, 1.0)}
    return w


def _run_phase2_once(stream_factory):
    cfg = load_config()
    w = _single_rover_world()
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    s = stream_factory(d.create_video_stream())
    u = fuwb.FakeUWBParserThread(world=w)
    st, tb = MissionState(), TaskBoard()
    worker = DroneWorker(d, u, 0, cfg, guard=ReactiveGuard(), sleep=NOSLEEP)
    worker.run_phase2([{"xy": (2.0, 1.0), "gimbal_deg": 90}], s, st, tb,
                      bubble=[(0, 0), (10, 0), (10, 2), (0, 2)], all_ids=[20],
                      intrinsics=INTR, budget_cycles=2, dwell_s=0.6)
    fpx.set_active_world(None)
    return st.tagged()


def test_worker_phase2_cold_stream_does_not_lose_the_rover():
    warm = _run_phase2_once(lambda inner: inner)                       # control: warm stream
    assert 20 in warm, "scenario must tag the rover when the stream is warm (control)"
    cold = _run_phase2_once(lambda inner: _ColdStartStream(inner, cold=30))  # ~3 s warm-up
    assert cold == warm                                               # warm-up did NOT cost the rover
