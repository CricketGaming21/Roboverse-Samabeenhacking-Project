"""Phase 17 acceptance test — winding loops, stationary observers, gradual
scripted lock-on (headless).

The lock-on is a SCRIPTED stand-in timed to a config-deterministic rover
pass — pitch and position must ease together, gradually, toward the rover's
approach. Route fidelity is re-guarded by the phase-11 full-path clearance
suite against the new branch coordinates.
"""

import math

import pytest

import scripts.scenario_demo as demo
from simcore import rover_model
from simcore.config import load_config


def _cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scenario.episode_seconds = 45.0
    c.scenario.ambush_seconds = 35.0
    c.scenario.deploy_timeout_s = 25.0
    c.scenario.ambush_trigger.mode = "on_all_landed"
    c.scenario.ambush_trigger.delay_s = 1.0
    return c


# --------------------------------------------------------------------------- #
# 1. The branches are long winding loops now
# --------------------------------------------------------------------------- #

def test_branches_are_winding_loops():
    cfg = _cfg()
    for i, branch in enumerate(cfg.rovers.convoy.branches):
        assert len(branch) >= 5, f"branch {i} is a short hop, not a loop"
        pts = [tuple(w) for w in branch]
        perimeter = (sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
                     + math.dist(pts[-1], pts[0]))  # loiter=loop closes it
        assert perimeter >= 4.0, f"branch {i} loop too small ({perimeter:.1f} m)"
    # clearance/staggering/bounds are enforced by the phase-11 suite against
    # these same configured routes — no duplicate checks here.


def test_pass_times_are_deterministic_config_geometry():
    cfg = _cfg()
    station = demo.OBSERVE_STATIONS[0]
    a = demo.rover_pass_times(cfg, 0, station, ambush_t0=10.0, horizon_s=120)
    b = demo.rover_pass_times(cfg, 0, station, ambush_t0=10.0, horizon_s=120)
    assert a == b and len(a) >= 2                  # pure arithmetic
    cycle = a[1] - a[0]
    assert all((t2 - t1) == pytest.approx(cycle, abs=1e-9)
               for t1, t2 in zip(a, a[1:]))        # one pass per loop cycle
    with pytest.raises(ValueError):                # stations must sit ON the route
        demo.rover_pass_times(cfg, 0, (3.3, 3.3), 0.0, 60)
    for i, station in enumerate(demo.OBSERVE_STATIONS):
        demo._station_on_route(cfg, demo.LOCKON_TARGETS[i], station)  # all valid


# --------------------------------------------------------------------------- #
# 2 + 3. Observers hold; the lock-on eases pitch + position together
# --------------------------------------------------------------------------- #

def test_demo_observers_hold_and_lockon_is_gradual():
    cfg = _cfg()
    result = demo.run_scenario_demo(cfg)
    assert result["final_phase"] == "done"
    assert result["landing_score"] >= 1

    samples = result["drone_samples"]              # (t, phase, [(n,e,alt,pitch)])
    station = demo.OBSERVE_STATIONS[0]
    adir = demo._approach_dir(cfg, demo.LOCKON_TARGETS[0], station)

    # --- observer 0 reaches its hover point and HOLDS (bounded) ---------- #
    ambush = [(t, dr[0]) for t, ph, dr in samples if ph == "ambush"]
    arrived = [(t, d) for t, d in ambush
               if d[2] > 1.0 and math.dist(d[:2], station) < 0.30]
    assert arrived, "drone 0 never reached its observation station"
    t_arrive = arrived[0][0]
    on_station = [(t, d) for t, d in ambush if t >= t_arrive]
    max_dev = max(math.dist(d[:2], station) for _t, d in on_station)
    assert max_dev <= 0.55, f"observer wandered {max_dev:.2f} m (not holding)"

    # --- the scripted gradual lock-on: pitch + position TOGETHER -------- #
    window = [(t, d) for t, d in on_station if d[3] < 89.0]
    assert window, "the scripted lock-on never ran"
    t_w0, t_w1 = window[0][0], window[-1][0]
    assert 1.0 <= t_w1 - t_w0 <= 5.0               # gradual, ~1-2 s, not a snap
    pitches = [d[3] for _t, d in window]
    assert len(set(pitches)) >= 4                  # eased through keyframes
    assert min(pitches) <= 70.0                    # tilted well off nadir
    full = [d[3] for _t, d in on_station]
    jumps = [abs(b - a) for a, b in zip(full, full[1:])]
    assert max(jumps) <= 16.0                      # smooth, never a snap

    # position eased WITH the pitch, toward the rover's approach
    devs = [(t, math.dist(d[:2], station), d) for t, d in window]
    t_max, dev_max, d_max = max(devs, key=lambda x: x[1])
    assert 0.08 <= dev_max <= 0.50                 # eased a little, bounded
    disp = (d_max[0] - station[0], d_max[1] - station[1])
    dot = (disp[0] * adir[0] + disp[1] * adir[1]) / dev_max
    assert dot >= 0.6                              # toward the approaching rover
    assert d_max[3] <= 85.0                        # pitch + position together

    # eased BACK afterwards: later samples re-centred at nadir
    after = [(t, d) for t, d in on_station if t > t_w1 + 1.0]
    if after:                                      # (episode may end first)
        t_a, d_a = after[-1]
        assert math.dist(d_a[:2], station) <= 0.30
        assert d_a[3] == 90.0


def test_lockon_is_a_fixed_script():
    assert demo.LOCKON_PITCH_SEQ[0] == demo.LOCKON_PITCH_SEQ[-1] == 90
    assert all(isinstance(p, (int, float)) for p in demo.LOCKON_PITCH_SEQ)
    deltas = [abs(b - a) for a, b in
              zip(demo.LOCKON_PITCH_SEQ, demo.LOCKON_PITCH_SEQ[1:])]
    assert max(deltas) <= 10                       # keyframes ease, not jump
    assert 0.05 <= demo.LOCKON_EASE_M <= 0.5
    assert len(demo.LOCKON_TARGETS) == 3
    # the no-sensing source pin lives in test_phase15_demo (forbidden imports)
