"""Phase 11 acceptance test — rover convoy routing (headless).

The 5 rovers ENTER from scenario.entrance (SW) staggered by entry_stagger_s,
follow the shared trunk to split_index, diverge onto DISTINCT branches, then
loiter. Fixed configured routes only — and they must THREAD BETWEEN the
authored crate clusters: the route-clearance test rejects any waypoint that
clips an obstacle footprint (nudge the YAML coordinate when it fires).
"""

import math
import time

import pytest

from simcore import arena, rover_model
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry

ROVER_RADIUS_M = 0.2     # half-diagonal of the rover footprint (0.16 x 0.12)
ROUTE_TOL_M = 0.35       # how far off the polyline a tracked rover may read
_SAMPLE_PITCH_M = 0.05   # route-clearance sampling pitch


def _cfg(**overrides):
    c = load_config("sim_config.yaml")  # authored map + convoy defaults
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.scenario.ambush_trigger.mode = "timed"
    c.scenario.ambush_trigger.delay_s = 0.5
    for key, value in overrides.items():
        setattr(c.rovers.convoy, key, value)
    return c


def _ground_obstacles(cfg):
    layout = arena.generate(cfg)
    rover_top = 2 * cfg.bodies.rover_half_extents_m[2] + 0.05
    return [o for o in layout.obstacles if o.z0_m < rover_top]


def _full_path(cfg, index):
    """Route polyline incl. the loiter-loop closing segment (if looping)."""
    route = rover_model.convoy_route(cfg, index)
    if cfg.rovers.convoy.loiter == "loop":
        first_branch = tuple(cfg.rovers.convoy.branches[index][0])
        route = route + [first_branch]  # branch end -> branch start
    return route

def _segment_points(a, b, pitch=_SAMPLE_PITCH_M):
    length = math.dist(a, b)
    n = max(2, int(length / pitch) + 1)
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
            for i in range(n + 1)]


def _point_to_polyline_m(p, polyline):
    best = float("inf")
    for a, b in zip(polyline, polyline[1:]):
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 == 0 else max(
            0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2))
        best = min(best, math.dist(p, (ax + t * dx, ay + t * dy)))
    return best


# --------------------------------------------------------------------------- #
# Pure route geometry: authored routes must thread the crates
# --------------------------------------------------------------------------- #

def test_routes_thread_between_the_crates():
    """No point of any route (incl. the loiter loop-back) may come within a
    rover radius of a ground obstacle footprint. If this fires after a config
    edit, nudge the waypoint in sim_config.yaml."""
    cfg = _cfg()
    obstacles = _ground_obstacles(cfg)
    L, W = cfg.arena.length_m, cfg.arena.width_m
    for index in range(cfg.rovers.count):
        route = _full_path(cfg, index)
        for a, b in zip(route, route[1:]):
            for pt in _segment_points(a, b):
                assert 0.2 <= pt[0] <= L - 0.2 and 0.2 <= pt[1] <= W - 0.2, \
                    f"rover {index}: route point {pt} too close to a wall"
                for o in obstacles:
                    d = arena.point_rect_dist_m(pt[0], pt[1], o.north, o.east,
                                                o.half_n, o.half_e)
                    assert d >= ROVER_RADIUS_M, (
                        f"rover {index}: route point ({pt[0]:.2f},{pt[1]:.2f})"
                        f" clips obstacle at ({o.north:.2f},{o.east:.2f}) "
                        f"(gap {d:.2f} m) — nudge the waypoint in "
                        f"sim_config.yaml")


def test_branches_distinct_and_route_shape():
    cfg = _cfg()
    cv = cfg.rovers.convoy
    ends = [tuple(b[-1]) for b in cv.branches[:cfg.rovers.count]]
    assert len(set(ends)) == cfg.rovers.count          # distinct branch ends
    for i in range(cfg.rovers.count):
        route = rover_model.convoy_route(cfg, i)
        assert route[0] == tuple(cfg.scenario.entrance)  # starts at the SW entrance
        shared = [tuple(w) for w in cv.trunk[:cv.split_index + 1]]
        assert route[1:1 + len(shared)] == shared        # same trunk for all
    with pytest.raises(ValueError):
        rover_model.convoy_route(cfg, 99)                # branch per rover


# --------------------------------------------------------------------------- #
# Live behaviour: staggered SW entry, trunk -> branch, on-route, loiter
# --------------------------------------------------------------------------- #

def test_staggered_entry_from_the_entrance():
    cfg = _cfg(entry_stagger_s=2.0)
    reg = get_registry(cfg)
    try:
        deadline = time.time() + 5.0
        while time.time() < deadline and reg.scenario.phase != "ambush":
            time.sleep(0.01)
        t0 = reg.scenario.ambush_started_at
        entries = {}        # rover index -> (sim time, first in-arena pos)
        while time.time() < deadline and len(entries) < cfg.rovers.count:
            for i, rover in enumerate(reg.rovers):
                if i not in entries and rover.in_arena:
                    entries[i] = (reg.sim_time(),
                                  reg.rover_arena_positions()[i])
            time.sleep(0.005)
        assert len(entries) == cfg.rovers.count, "not all rovers entered"
        for i in range(cfg.rovers.count):
            t_entry, (n, e) = entries[i]
            # rover k starts ~k * entry_stagger_s after AMBUSH begins
            assert t_entry - t0 == pytest.approx(i * 2.0, abs=0.6), \
                f"rover {i} entered at +{t_entry - t0:.2f}s"
            # ...and AT the entrance — never teleported mid-arena
            assert math.dist((n, e), tuple(cfg.scenario.entrance)) < 0.35, \
                f"rover {i} appeared at ({n:.2f},{e:.2f})"
    finally:
        shutdown_registry()


def test_trunk_then_branch_on_route_and_loiter():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        split = tuple(cfg.rovers.convoy.trunk[cfg.rovers.convoy.split_index])
        obstacles = _ground_obstacles(cfg)
        paths = [_full_path(cfg, i) for i in range(cfg.rovers.count)]
        tracks = [[] for _ in range(cfg.rovers.count)]
        # ~55 sim seconds: every rover enters, traverses, starts loitering
        for _ in range(275):
            now = reg.sim_time()
            for i, (pos, rover) in enumerate(zip(reg.rover_arena_positions(),
                                                 reg.rovers)):
                if rover.in_arena:
                    tracks[i].append((now, pos))
            time.sleep(0.02)

        for i, track in enumerate(tracks):
            assert track, f"rover {i} never entered"
            pts = [p for _t, p in track]
            # stays ON its configured route the entire run...
            worst = max(_point_to_polyline_m(p, paths[i]) for p in pts)
            assert worst <= ROUTE_TOL_M, f"rover {i} strayed {worst:.2f} m"
            # ...inside the arena...
            assert all(0 <= n <= 10 and 0 <= e <= 6 for n, e in pts)
            # ...and never into a crate footprint
            for n, e in pts:
                for o in obstacles:
                    assert arena.point_rect_dist_m(
                        n, e, o.north, o.east, o.half_n, o.half_e) >= 0.15
            # trunk in order: reaches the split point BEFORE the branch end
            t_split = next(t for t, p in track if math.dist(p, split) < 0.3)
            branch_end = paths[i][-2 if cfg.rovers.convoy.loiter == "loop"
                                  else -1]
            t_end = next(t for t, p in track
                         if math.dist(p, branch_end) < 0.3)
            assert t_split < t_end, f"rover {i} skipped the trunk"
            # diverged onto ITS branch and loiters there at the end
            branch = [tuple(w) for w in cfg.rovers.convoy.branches[i]]
            late = pts[-20:]
            assert all(_point_to_polyline_m(p, branch + [branch[0]])
                       <= ROUTE_TOL_M for p in late), \
                f"rover {i} not loitering on its own branch"
        # distinct branches: late positions are spread, not bunched
        finals = [t[-1][1] for t in tracks]
        spreads = [math.dist(a, b) for k, a in enumerate(finals)
                   for b in finals[k + 1:]]
        assert max(spreads) > 2.0
    finally:
        shutdown_registry()


def test_loiter_hold_parks_at_branch_end():
    cfg = _cfg(loiter="hold", entry_stagger_s=0.5)
    reg = get_registry(cfg)
    try:
        time.sleep(3.5)  # ~35 sim s: rover 0 finishes its ~7 m route
        end = tuple(cfg.rovers.convoy.branches[0][-1])
        p1 = reg.rover_arena_positions()[0]
        assert math.dist(p1, end) < 0.1, "rover 0 should hold at its branch end"
        time.sleep(0.3)
        assert reg.rover_arena_positions()[0] == p1  # genuinely parked
    finally:
        shutdown_registry()


def test_patrol_mode_retained_behind_flag():
    cfg = _cfg()
    cfg.rovers.motion = "patrol"
    cfg.scenario.phases = "ambush"
    reg = get_registry(cfg)
    try:
        n_lo, n_hi = cfg.rovers.patrol.bounds_north
        e_lo, e_hi = cfg.rovers.patrol.bounds_east
        p0 = reg.rover_arena_positions()
        time.sleep(0.5)
        p1 = reg.rover_arena_positions()
        assert p0 != p1                                # random patrol wanders
        assert all(n_lo - 0.05 <= n <= n_hi + 0.05
                   and e_lo - 0.05 <= e <= e_hi + 0.05 for n, e in p1)
    finally:
        shutdown_registry()


def test_convoy_is_deterministic_config_no_rng():
    """Convoy routing has NO randomness: routes and entry times are pure
    config, independent of the seed."""
    cfg_a, cfg_b = _cfg(), _cfg()
    cfg_b.meta.seed = 1234
    for i in range(cfg_a.rovers.count):
        assert (rover_model.convoy_route(cfg_a, i)
                == rover_model.convoy_route(cfg_b, i))
    reg = get_registry(cfg_b)  # different seed: same staggered entries
    try:
        deadline = time.time() + 5.0
        while time.time() < deadline and not reg.rovers[1].in_arena:
            time.sleep(0.01)
        assert reg.rovers[1]._entry_time == pytest.approx(
            reg.scenario.ambush_started_at
            + cfg_b.rovers.convoy.entry_stagger_s)
    finally:
        shutdown_registry()
