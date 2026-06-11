"""Mission orchestrator — spawn a DroneWorker per drone, run Phase 1 → Phase 2, monitor
for worker death and **reassign a dead drone's zone**, and **always land every drone on
shutdown** (in a `finally`). One drone failing must never freeze the others (HARD invariant #6).

Deterministic sequential execution is the default (used by the fake-harness integration test);
`parallel=True` runs a thread per drone for real deployment (blocking inside a worker is fine).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from mission.control.avoidance import ReactiveGuard
from mission.mission.worker import DroneWorker, WorkerState
from mission.mission.phase2_search import phase2_search
from mission.runtime import sdk_compat
from mission.world.mission_state import MissionState
from mission.world.taskboard import TaskBoard

Point = Tuple[float, float]


@dataclass
class MissionResult:
    landings: int
    tagged: set
    landings_map: Dict[int, bool] = field(default_factory=dict)
    workers: Dict[int, DroneWorker] = field(default_factory=dict)


class Mission:
    def __init__(self, cfg, plan, *, drones: Dict[int, object], uwb,
                 streams: Optional[Dict[int, object]] = None,
                 pad_coords: Optional[Dict[int, Point]] = None,
                 footprints: Sequence = (), all_rover_ids: Sequence[int] = (),
                 intrinsics=None, state: Optional[MissionState] = None,
                 taskboard: Optional[TaskBoard] = None, sleep=time.sleep,
                 phase2_kwargs: Optional[dict] = None):
        self.cfg = cfg
        self.plan = plan
        self.tags = sorted(drones)
        self.pad_coords = dict(pad_coords) if pad_coords else {}
        self.all_rover_ids = list(all_rover_ids)
        self.intrinsics = intrinsics
        self.state = state or MissionState()
        self.taskboard = taskboard or TaskBoard()
        self.sleep = sleep
        self.phase2_kwargs = dict(phase2_kwargs or {})
        streams = streams or {}
        self.workers: Dict[int, DroneWorker] = {
            tag: DroneWorker(drones[tag], uwb, tag, cfg, guard=ReactiveGuard(),
                             footprints=footprints, stream=streams.get(tag),
                             priority=tag, sleep=sleep,
                             battery_rtl_pct=cfg.failsafe.battery_rtl_pct)
            for tag in self.tags}

    # -- per-worker steps ------------------------------------------------- #
    def _phase1_one(self, w: DroneWorker) -> bool:
        pad_id = self.plan.pad_assignment()[w.tag_id]
        pad_xy = self.pad_coords[pad_id]
        return w.run_phase1(pad_xy, self.plan.route(w.tag_id))

    def _phase2_one(self, w: DroneWorker) -> set:
        return w.run_phase2(self.plan.vantages(w.tag_id), w.stream, self.state,
                            self.taskboard, bubble=self.plan.bubble(w.tag_id),
                            all_ids=self.all_rover_ids, intrinsics=self.intrinsics,
                            **self.phase2_kwargs)

    def _run_workers(self, fn, parallel: bool) -> Dict[int, object]:
        results: Dict[int, object] = {}
        if parallel:
            def runner(tag):
                try:
                    results[tag] = fn(self.workers[tag])
                except Exception as exc:                 # worker death must not propagate
                    results[tag] = exc
            threads = [threading.Thread(target=runner, args=(t,), daemon=True)
                       for t in self.tags]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
        else:
            for tag in self.tags:
                try:
                    results[tag] = fn(self.workers[tag])
                except Exception as exc:
                    results[tag] = exc
        return results

    # -- phases ----------------------------------------------------------- #
    def run_phase1(self, parallel: bool = False) -> Dict[int, bool]:
        res = self._run_workers(self._phase1_one, parallel)
        return {t: (r is True) for t, r in res.items()}

    def run_phase2(self, parallel: bool = False) -> set:
        self._run_workers(self._phase2_one, parallel)
        self._reassign_dead_zones()                      # cover any dead drone's zone
        return self.state.tagged()

    def _reassign_dead_zones(self) -> None:
        dead = [t for t in self.tags if self.workers[t].state == WorkerState.FAILED]
        alive = [t for t in self.tags if self.workers[t].state != WorkerState.FAILED]
        if not alive:
            return
        for dtag in dead:
            helper = self.workers[alive[0]]              # nearest-free would be nicer; first alive is fine
            remaining = set(self.all_rover_ids) - self.state.tagged()
            if not remaining:
                break
            try:                                         # mop-up the dead drone's vantages, gate off
                helper.run_phase2(self.plan.vantages(dtag), helper.stream, self.state,
                                  self.taskboard, bubble=None, all_ids=self.all_rover_ids,
                                  intrinsics=self.intrinsics, **self.phase2_kwargs)
            except Exception:
                pass

    # -- shutdown / run --------------------------------------------------- #
    def shutdown(self) -> None:
        """Land EVERY drone, always (called in run()'s finally)."""
        for w in self.workers.values():
            try:
                w.drone.land()
            except Exception:
                pass
            try:
                sdk_compat.release(w.drone)
            except Exception:
                pass

    def run(self, parallel: bool = False) -> MissionResult:
        try:
            landings = self.run_phase1(parallel)
            tagged = self.run_phase2(parallel)
            return MissionResult(landings=sum(1 for ok in landings.values() if ok),
                                 tagged=set(tagged), landings_map=landings,
                                 workers=self.workers)
        finally:
            self.shutdown()


# --------------------------------------------------------------------------- #
# Runnable entrypoint: python -m mission.runtime.main  (against the sim/real)
# --------------------------------------------------------------------------- #
class _PlanView:
    """Duck-typed plan the Mission consumes — routes auto-planned, vantages a general
    overwatch grid (the mission does NOT know the convoy routes on the real day)."""

    def __init__(self, pad_ids, routes, vantages):
        self._pad_ids = pad_ids
        self._routes = routes
        self._vantages = vantages

    def pad_assignment(self):
        return dict(self._pad_ids)

    def route(self, tag):
        return list(self._routes[tag])

    def vantages(self, tag):
        return list(self._vantages[tag])

    def bubble(self, tag):
        return None                      # no zone-gating for the baseline convoy run


def _nn_order(start, pts):
    """Nearest-neighbour tour from `start` over `pts` (cheap patrol ordering)."""
    import math
    remaining = list(pts)
    out, cur = [], start
    while remaining:
        nxt = min(remaining, key=lambda p: math.hypot(p[0] - cur[0], p[1] - cur[1]))
        out.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    return out


def overwatch_vantages(bounds, inflated, pad_by_tag, *, gimbal_deg=90.0, dwell_s=1.0,
                       step_n=1.6, step_e=1.4):
    """A grid of nadir overwatch points clear of inflated footprints, assigned to the
    nearest drone's pad and ordered into a patrol. General search — not convoy-tuned."""
    import math

    from mission.planner.geometry import point_blocked
    pts = []
    n = bounds.min_n + 1.5
    while n <= bounds.max_n - 1.0:
        e = bounds.min_e + 1.0
        while e <= bounds.max_e - 0.8:
            p = (round(n, 2), round(e, 2))
            if not point_blocked(p, inflated):          # keep the drone off footprints
                pts.append(p)
            e += step_e
        n += step_n
    groups = {tag: [] for tag in pad_by_tag}
    for p in pts:
        tag = min(pad_by_tag, key=lambda t: math.hypot(p[0] - pad_by_tag[t][0],
                                                       p[1] - pad_by_tag[t][1]))
        groups[tag].append(p)
    return {tag: [{"xy": p, "look_yaw_deg": 0, "gimbal_deg": gimbal_deg,
                   "dwell_s": dwell_s}
                  for p in _nn_order(pad_by_tag[tag], gp)]
            for tag, gp in groups.items()}


def _read_starts_from_uwb(uwb, tags, sleep, *, attempts=60, delay=0.1):
    """Real-day drone starts come from UWB (no configured starts). Retry until each tag has
    a fix; raise with a clear message if a tag never appears (cage/origin/tag-id problem)."""
    starts = {}
    for _ in range(max(1, attempts)):
        for t in tags:
            if t not in starts:
                x, y, _t = uwb.get_tag_position(t)
                if x is not None and y is not None:
                    starts[t] = (x, y)
        if len(starts) == len(tags):
            break
        sleep(delay)
    missing = [t for t in tags if t not in starts]
    if missing:
        raise SystemExit(f"UWB: no fix for tag(s) {missing} after {attempts} tries — "
                         f"check cage power / uwb.origin_x|y / tag ids")
    return starts


def build_live_mission(cfg, *, sleep, use_dola=False, real=False):
    """Wire discovery → connect → UWB → workers for a live (sim/real) run.

    real=True: Dola-discover IPs and map them to the configured tag_ids in order; start UWB
    with the per-cage origin; read each drone's start from UWB (no configured starts)."""
    from pathlib import Path

    import pyhulax
    from UWBParserThread import UWBParserThread

    from mission.planner.arena import load_arena
    from mission.planner.geometry import Rect, build_graph, inflate, plan_path
    from mission.planner.projection import CameraIntrinsics
    from mission.mission.phase1_land import assign_pads
    from mission.runtime.discovery import Discovery

    arena_path = Path(cfg.planner.arena_truth_file)
    if not arena_path.is_absolute():                          # resolve relative to repo root
        arena_path = Path(__file__).resolve().parents[3] / cfg.planner.arena_truth_file
    arena = load_arena(arena_path)
    bounds = Rect(0.0, 0.0, arena.length_m, arena.width_m)
    footprints = arena.footprint_tuples()
    inflated = inflate(footprints, cfg.planner.inflate_m)
    # reduced margin so a pad hard against a thin obstacle (e.g. arch post) is reachable via a
    # raw-clear final approach; interior routing stays on the full inflation
    approach = inflate(footprints, cfg.planner.pad_approach_inflate_m)
    graph = build_graph(inflated, bounds, approach=approach)
    # vantages get EXTRA clearance so UWB noise + lock-on wander can't drift the drone
    # over a crate (ToF altitude-hold over a crate climbs and breaches the altitude cap).
    vant_inflated = inflate(footprints, cfg.planner.inflate_m + 0.3)

    disc = Discovery.from_config(cfg, use_dola=(cfg.use_dola() or use_dola))
    ips = disc.resolve_ordered(log=print) if real else disc.resolve()    # {tag: ip}

    drones, streams = {}, {}
    for tag in sorted(ips):
        d = pyhulax.DroneAPI()
        d.connect(ips[tag])
        sdk_compat.prepare_manual_control(d, velocity_level=None)
        s = d.create_video_stream()      # created now; camera enabled at Phase-2 start (R1)
        drones[tag], streams[tag] = d, s
    uwb = UWBParserThread(x_origin=cfg.uwb.origin_x, y_origin=cfg.uwb.origin_y)
    uwb.start()

    starts = cfg.starts_by_tag()                             # sim: configured starts
    if real or len(starts) < len(drones):                   # real: read starts from UWB
        starts = _read_starts_from_uwb(uwb, sorted(drones), sleep)

    designated = cfg.designated_pads()
    assignment = assign_pads(designated, {t: starts[t] for t in drones})   # {tag: PadCfg}
    pad_ids = {t: assignment[t].id for t in assignment}
    pad_coords = {assignment[t].id: (assignment[t].north, assignment[t].east)
                  for t in assignment}
    pad_by_tag = {t: pad_coords[pad_ids[t]] for t in assignment}

    routes = {}
    for t in sorted(drones):
        path = plan_path(starts[t], pad_by_tag[t], graph)
        routes[t] = path if path else [starts[t], pad_by_tag[t]]

    vantages = overwatch_vantages(bounds, vant_inflated, pad_by_tag,
                                  gimbal_deg=search_pitch_deg(cfg))
    plan = _PlanView(pad_ids, routes, vantages)
    intr = CameraIntrinsics(cfg.camera.width, cfg.camera.height, cfg.camera.h_fov_deg)
    return drones, streams, uwb, pad_coords, footprints, intr, plan, starts, graph, bounds


def search_pitch_deg(cfg) -> float:
    """The held Phase-2 camera pitch: a MODERATE forward tilt for the gimbal, or the 90° nadir
    baseline when `camera.use_nadir_search` is set (a selectable fallback)."""
    return 90.0 if cfg.camera.use_nadir_search else cfg.camera.search_pitch_deg


def r2_phase2_kwargs(cfg, *, graph=None, footprints=(), bounds=None,
                     evidence_dir="logs/evidence") -> dict:
    """Phase-2 search/read kwargs for the MOVING-GIMBAL arena (R2 + R4): held search pitch, a
    body presence detector (so a drone persists on a seen-but-unread rover), the persist/orbit
    budget, and the **R4 evidence writer** (one annotated PNG per banked id + a gallery) — all
    config-driven. Shared by the live runner and the real-sim integration test."""
    from mission.perception.detector import ClassicalRoverDetector
    from mission.perception.evidence import EvidenceWriter
    return {
        "gimbal_deg": search_pitch_deg(cfg),
        "graph": graph,
        "footprints": list(footprints),
        "bounds": bounds,
        "presence": ClassicalRoverDetector(min_area_px=cfg.search.presence_min_area_px),
        "persist_timeout_s": cfg.search.persist_timeout_s,
        "orbit_step_m": cfg.search.orbit_step_m,
        "max_orbits": cfg.search.max_orbits,
        "presence_min_area_px": cfg.search.presence_min_area_px,
        "evidence": EvidenceWriter(evidence_dir),     # R4: judge deliverable, one PNG per id
    }


def _report(cfg, mission, pad_coords, plan, land_xy, footprints, rover_ids):
    """Mission-side honest report (the authoritative referee lives in the sim's
    DebugProbe, which mission code may not import — the @integration test cross-checks it).
    `land_xy` = {tag: (x,y)} captured from UWB right after Phase 1 (before shutdown)."""
    import math
    tol = cfg.landing.hoop_tol_m
    lines = ["", "=" * 64, "MISSION REPORT (mission-side telemetry)", "=" * 64]
    pad_ids = plan.pad_assignment()
    landed = 0
    for tag in sorted(mission.workers):
        w = mission.workers[tag]
        pad_id = pad_ids[tag]
        px, py = pad_coords[pad_id]
        xy = land_xy.get(tag)
        if xy is None or xy[0] is None:
            lines.append(f"  drone {tag}: pad {pad_id}  UWB=DROPOUT  state={w.state.value}")
            continue
        err = math.hypot(xy[0] - px, xy[1] - py)
        landed += int(w.landed_ok)                            # robust averaged determination
        lines.append(f"  drone {tag}: pad {pad_id} phase1_land_err≈{err * 100:5.1f}cm "
                     f"{'IN-HOOP' if w.landed_ok else 'OUT'}  final_state={w.state.value}  "
                     f"err={w.error}")
    tagged = sorted(mission.state.tagged())
    distinct = sorted(set(tagged) & set(rover_ids))
    lines.append("-" * 64)
    lines.append(f"  LANDINGS in-hoop ({tol:.2f} m): {landed}/{len(mission.workers)}")
    lines.append(f"  STAGE-2 distinct rover ids tagged: {distinct}  "
                 f"({len(distinct)}/{len(rover_ids)})")
    lines.append(f"  all banked ids (incl. non-convoy): {tagged}")
    # self-checked compliance: any trace point over a raw crate footprint
    viol = []
    for tag, w in mission.workers.items():
        for p in w.trace:
            if any(abs(p[0] - cn) < sn / 2 and abs(p[1] - ce) < se / 2
                   for cn, ce, sn, se in footprints):
                viol.append(tag)
                break
    lines.append(f"  self-checked over-crate compliance flags: "
                 f"{sorted(set(viol)) if viol else 'none'}")
    lines.append("=" * 64)
    print("\n".join(lines), flush=True)


def _run(cfg, *, real, sleep, cycles, dwell, rover_ids, use_dola=False,
         evidence_dir="logs/evidence", phase_budget_s=None, log=print) -> int:
    """Discover+connect, UWB (cage origin), Phase 1 land, Phase 2 search — landing every
    drone in a `finally`, holding on UWB dropout, Ctrl-C → abort-and-land. Returns 0."""
    import time

    log(f"[main] mode: {'REAL hardware' if real else 'sim'} — discovering + connecting…")
    drones, streams, uwb, pad_coords, footprints, intr, plan, starts, graph, bounds = \
        build_live_mission(cfg, sleep=sleep, use_dola=use_dola, real=real)

    pad_ids = plan.pad_assignment()
    for tag in sorted(drones):                                # per-drone status line
        x, y, _t = uwb.get_tag_position(tag)
        pid = pad_ids[tag]
        pad = pad_coords[pid]
        pos = f"({x:.2f},{y:.2f})" if x is not None else "DROPOUT"
        log(f"  drone tag {tag}: connected  UWB={pos}  -> pad {pid} "
            f"@ ({pad[0]:.2f},{pad[1]:.2f})  state=INIT")

    t0 = time.time()                                              # mission start — evidence shows
    mission_clock = lambda: time.time() - t0                      # ELAPSED s (not the raw epoch)
    phase2_kwargs = {"budget_cycles": cycles, "dwell_s": dwell, "mopup_extra_cycles": 1,
                     "rover_ids": rover_ids,                        # allow-list (config)
                     "dictionary": cfg.aruco.dictionary,
                     "clock": mission_clock,                       # R4 evidence caption = elapsed s
                     **r2_phase2_kwargs(cfg, graph=graph,           # held search pitch + gimbal
                                        footprints=footprints, bounds=bounds,
                                        evidence_dir=evidence_dir)}   # persistence + R4 evidence
    if phase_budget_s is not None:                                # else worker uses cfg cap (180 s)
        phase2_kwargs["phase_budget_s"] = phase_budget_s          # hard Phase-2 wall-clock cap
    mission = Mission(cfg, plan, drones=drones, uwb=uwb, streams=streams,
                      pad_coords=pad_coords, footprints=footprints,
                      all_rover_ids=rover_ids, intrinsics=intr, sleep=sleep,
                      phase2_kwargs=phase2_kwargs)
    land_xy = {}
    try:
        landings = mission.run_phase1(parallel=True)
        land_xy = {t: uwb.get_tag_position(t)[:2] for t in drones}   # capture before relaunch
        for tag in sorted(mission.workers):
            w = mission.workers[tag]
            log(f"  drone tag {tag}: PHASE-1 "
                f"{'LANDED' if landings.get(tag) else 'NOT-LANDED'}  state={w.state.value}")
        if not real:
            log("[main] waiting for convoy (on_all_landed trigger)…")
            time.sleep(4.0)                          # sim scenario wait (real time)
        mission.run_phase2(parallel=True)
    except KeyboardInterrupt:
        log("\n[main] Ctrl-C — ABORTING: landing all drones…")
    finally:
        mission.shutdown()                           # lands every drone
        try:
            uwb.stop()
        except Exception:
            pass
        for d in drones.values():
            sdk_compat.release(d)
    ew = phase2_kwargs.get("evidence")               # R4: rebuild the final gallery from state
    if ew is not None:
        try:
            idx = ew.gallery(mission.state.evidence())
            log(f"[main] evidence: {mission.state.count()} annotated capture(s) + gallery → {idx}")
        except Exception as exc:
            log(f"[main] evidence gallery failed: {exc!r}")
    _report(cfg, mission, pad_coords, plan, land_xy, footprints, rover_ids)
    return 0


def main(argv=None) -> int:
    import argparse
    import os
    import time

    from mission.config import load_config, load_real_config

    ap = argparse.ArgumentParser(
        description="HULA mission runner — sim by default, --real for cage hardware.")
    ap.add_argument("--real", action="store_true",
                    help="real-hardware profile (config/mission_real.yaml + Dola discovery + "
                         "UWB cage origin)")
    args = ap.parse_args(argv)

    cfg = load_real_config() if args.real else load_config()
    env_ids = os.environ.get("HULA_ROVER_IDS")           # config-driven; env override optional
    rover_ids = ([int(x) for x in env_ids.split(",")] if env_ids
                 else list(cfg.aruco.rover_ids))
    cycles = int(os.environ.get("HULA_PHASE2_CYCLES", "3"))
    dwell = float(os.environ.get("HULA_PHASE2_DWELL_S", "1.0"))
    use_dola = bool(os.environ.get("HULA_USE_DOLA"))
    budget_env = os.environ.get("HULA_PHASE2_BUDGET_S")  # override the cfg wall-clock cap (e.g. a
    phase_budget_s = float(budget_env) if budget_env else None   # GPU-slow sim where rtf << 1)
    try:
        return _run(cfg, real=args.real, sleep=time.sleep, cycles=cycles, dwell=dwell,
                    rover_ids=rover_ids, use_dola=use_dola, phase_budget_s=phase_budget_s)
    except KeyboardInterrupt:                         # before the run loop owns it
        print("\n[main] Ctrl-C before launch — exiting.")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
