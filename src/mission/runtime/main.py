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
