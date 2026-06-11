"""Real-hardware entrypoint wiring (--real), against the fake substrate.

Verifies the real path: Dola-ordered discovery, UWB started with the cage origin, drone
starts read from UWB, the 3 designated real pads assigned + landed, and `_run` lands all
in a `finally`. Does NOT need the real sim — the fakes stand in.
"""

import math

import pytest
import yaml

from mission.config import load_real_config
from mission.runtime.main import _run, build_live_mission, Mission
from tests.fakes import fake_pyhulax as fpx

_ROOT = __import__("pathlib").Path(__file__).resolve().parent.parent
NOSLEEP = lambda _s: None


def _cage_world():
    """An open 11×11 cage (no crates), 3 grounded drones near the origin, no rovers."""
    w = fpx.FakeWorld(length_m=11.0, width_m=11.0, crates=[])
    w.pads = []
    w.rovers = []
    w.drone_starts = {0: (0.6, 0.6), 1: (1.0, 0.6), 2: (0.6, 1.0)}
    fpx.set_active_world(w)
    return w


# --------------------------------------------------------------------------- #
# build wiring + landing on the real pads
# --------------------------------------------------------------------------- #
def test_real_build_wiring_and_lands_3of3():
    w = _cage_world()
    cfg = load_real_config()
    (drones, streams, uwb, pad_coords, footprints, intr, plan, starts, graph, bounds) = \
        build_live_mission(cfg, sleep=NOSLEEP, real=True)
    assert sorted(drones) == [0, 1, 2]                       # discovered + connected, in order
    # starts came from UWB (== where the drones physically sit), not config
    for t in (0, 1, 2):
        assert starts[t] == pytest.approx(w.drone_starts[t], abs=0.05)
    # the 3 DESIGNATED real pads got assigned
    assert set(plan.pad_assignment().values()) == {11, 51, 101}

    mission = Mission(cfg, plan, drones=drones, uwb=uwb, streams=streams,
                      pad_coords=pad_coords, footprints=footprints, all_rover_ids=[20],
                      intrinsics=intr, sleep=NOSLEEP, phase2_kwargs={"budget_cycles": 0})
    landings = mission.run_phase1(parallel=False)
    assert sum(landings.values()) == 3
    for t in (0, 1, 2):
        pid = plan.pad_assignment()[t]
        px, py = pad_coords[pid]
        x, y, _ = uwb.get_tag_position(t)
        assert math.hypot(x - px, y - py) <= cfg.landing.hoop_tol_m   # landed in the hoop
    mission.shutdown()
    uwb.stop()
    fpx.set_active_world(None)


def test_real_build_passes_uwb_cage_origin(tmp_path):
    _cage_world()
    data = yaml.safe_load((_ROOT / "config" / "mission_real.yaml").read_text())
    data["uwb"]["origin_x"] = 5.5                            # Cage2/3 origin
    data["uwb"]["origin_y"] = 5.5
    p = tmp_path / "real.yaml"
    p.write_text(yaml.safe_dump(data))
    cfg = load_real_config(p)
    assert cfg.uwb.origin_x == 5.5
    (_d, _s, uwb, *_rest) = build_live_mission(cfg, sleep=NOSLEEP, real=True)
    assert uwb.origin_x == 5.5 and uwb.origin_y == 5.5       # origin reached UWBParserThread
    uwb.stop()
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# _run end-to-end (lands all, prints status), real mode
# --------------------------------------------------------------------------- #
def test_real_run_returns_zero_and_lands(capsys):
    _cage_world()
    rc = _run(load_real_config(), real=True, sleep=NOSLEEP, cycles=0, dwell=0.6,
              rover_ids=[20])
    assert rc == 0
    out = capsys.readouterr().out
    assert "REAL hardware" in out
    assert "LANDINGS in-hoop (0.20 m): 3/3" in out          # real hoop tolerance
    assert "drone tag 0" in out                             # per-drone status lines
    fpx.set_active_world(None)
