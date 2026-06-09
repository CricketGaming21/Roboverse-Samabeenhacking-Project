"""Test bootstrap: put `src` + repo-root on the path, install the fake `pyhulax`
and `UWBParserThread` modules into `sys.modules` (so the mission's real imports
resolve to the fakes), seed RNGs, and expose world/drone/uwb fixtures.

Integration tests (`@pytest.mark.integration`) need the REAL sim and are excluded
from every gate / the overnight loop — they are skipped here unless RUN_INTEGRATION
is set. This is the documented mechanism (CLAUDE.md / PHASE_PLAN.md), not test weakening.
"""

from __future__ import annotations

import os
import random
import sys
import types
from pathlib import Path

import numpy as np
import pytest

# --- paths: make `mission` (src layout) and `tests.fakes` importable --------- #
_ROOT = Path(__file__).resolve().parent.parent
for _p in (str(_ROOT), str(_ROOT / "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from tests.fakes import fake_pyhulax as fpx          # noqa: E402
from tests.fakes import fake_uwb as fuwb             # noqa: E402

# --- build & install the fake pyhulax package ------------------------------- #
_pyhulax = types.ModuleType("pyhulax")
_pyhulax.DroneAPI = fpx.DroneAPI
_pyhulax.CommandResult = fpx.CommandResult

_core = types.ModuleType("pyhulax.core")
for _name in ("Direction", "CameraPitchMode", "VelocityLevel", "BarrierMask",
              "TakeoffFlags", "VideoResolution", "Vector3", "Orientation",
              "Obstacles", "CommandResult"):
    setattr(_core, _name, getattr(fpx, _name))
_pyhulax.core = _core

_video = types.ModuleType("pyhulax.video")
_video.VideoStream = fpx.FakeVideoStream
_video.VideoFrame = fpx.FakeVideoFrame
_pyhulax.video = _video

_discovery = types.ModuleType("pyhulax.discovery")
_discovery.Dola = fpx.FakeDola
_pyhulax.discovery = _discovery

sys.modules["pyhulax"] = _pyhulax
sys.modules["pyhulax.core"] = _core
sys.modules["pyhulax.video"] = _video
sys.modules["pyhulax.discovery"] = _discovery

_uwb_mod = types.ModuleType("UWBParserThread")
_uwb_mod.UWBParserThread = fuwb.FakeUWBParserThread
sys.modules["UWBParserThread"] = _uwb_mod


# --- collection: skip integration unless explicitly requested --------------- #
def pytest_collection_modifyitems(config, items):
    if os.environ.get("RUN_INTEGRATION"):
        return
    skip = pytest.mark.skip(reason="integration: needs the real sim "
                                   "(excluded from gates; set RUN_INTEGRATION=1)")
    for item in items:
        if "integration" in item.keywords:
            item.add_marker(skip)


# --- fixtures --------------------------------------------------------------- #
@pytest.fixture(autouse=True)
def _seed_everything():
    """Deterministic RNG for every test."""
    random.seed(42)
    np.random.seed(42)
    yield


@pytest.fixture
def world():
    """A standard FakeWorld, registered as the active world for bare DroneAPI()."""
    w = fpx.default_world()
    fpx.set_active_world(w)
    yield w
    fpx.set_active_world(None)


@pytest.fixture
def make_drone(world):
    """Factory: make_drone(tag_id=0) -> a connected FakeDroneAPI bound to `world`."""
    def _make(tag_id: int = 0, real_like: bool = False):
        cls = fpx.RealLikeFakeDroneAPI if real_like else fpx.FakeDroneAPI
        d = cls(world)
        d.connect(world.ip_map[tag_id])
        return d
    return _make


@pytest.fixture
def drone(make_drone):
    """A single connected drone (tag 0)."""
    return make_drone(0)


@pytest.fixture
def uwb(world):
    """A FakeUWBParserThread bound to `world` (noiseless by default)."""
    u = fuwb.FakeUWBParserThread(world=world)
    u.start()
    yield u
    u.stop()
