"""DroneWorker — the per-drone state machine (one thread per drone in the mission).

Phase-1 states: INIT → TAKEOFF → GO_TO_PAD → LAND_HOOP → LANDED (or FAILED).
Blocking inside a worker is fine; an exception is caught and the drone is landed so one
drone's failure never freezes the others (HARD invariant #6). Phase-2 states
(RELAUNCH/SEARCH/HOME/CONVERGE) are added in P7.
"""

from __future__ import annotations

import time
from enum import Enum
from typing import List, Optional, Sequence, Tuple

from mission.control.uwb_loop import fly_to_uwb
from mission.frames import m_to_cm
from mission.mission.phase1_land import land_in_hoop
from mission.planner.geometry import plan_path
from mission.runtime import sdk_compat

Point = Tuple[float, float]


class WorkerState(str, Enum):
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    GO_TO_PAD = "GO_TO_PAD"
    LAND_HOOP = "LAND_HOOP"
    LANDED = "LANDED"
    FAILED = "FAILED"


def _loop_kwargs(cfg) -> dict:
    return dict(kp_xy=cfg.speed.kp_xy, kp_alt=cfg.speed.kp_alt,
                max_mps=cfg.speed.max_mps, climb_mps=cfg.speed.climb_mps,
                yaw_offset_deg=cfg.frame.yaw_offset_deg,
                invert_forward=cfg.frame.invert_forward,
                invert_right=cfg.frame.invert_right,
                rate_hz=cfg.speed.ctrl_rate_hz,
                hold_on_dropout=cfg.uwb.hold_on_dropout)


class DroneWorker:
    def __init__(self, drone, uwb, tag_id: int, cfg, *, guard=None, graph=None,
                 footprints: Optional[Sequence] = None, stream=None,
                 priority: int = 0, sleep=time.sleep):
        self.drone = drone
        self.uwb = uwb
        self.tag_id = tag_id
        self.cfg = cfg
        self.guard = guard
        self.graph = graph
        self.footprints = footprints
        self.stream = stream
        self.priority = priority
        self.sleep = sleep
        self.state = WorkerState.INIT
        self.history: List[WorkerState] = [WorkerState.INIT]
        self.trace: List[Point] = []
        self.landed_ok = False
        self.error: Optional[str] = None

    # -- helpers ---------------------------------------------------------- #
    def _set(self, s: WorkerState) -> None:
        self.state = s
        self.history.append(s)

    def _on_step(self, info: dict) -> None:
        self.trace.append((self.drone.n, self.drone.e))

    def _current_xy(self) -> Point:
        x, y, _ = self.uwb.get_tag_position(self.tag_id)
        if x is None or y is None:
            return (self.drone.n, self.drone.e)
        return (x, y)

    def _route_to(self, pad_xy: Point, route: Optional[Sequence[Point]]) -> List[Point]:
        if route is not None:
            return [tuple(w) for w in route]
        if self.graph is not None:
            path = plan_path(self._current_xy(), pad_xy, self.graph)
            if path is not None:
                return path[1:] if len(path) > 1 else path
        return [pad_xy]

    # -- phase 1 ---------------------------------------------------------- #
    def run_phase1(self, pad_xy: Point, route: Optional[Sequence[Point]] = None) -> bool:
        """Walk INIT→TAKEOFF→GO_TO_PAD→LAND_HOOP. Returns True iff landed in the hoop."""
        cfg = self.cfg
        lk = _loop_kwargs(cfg)
        try:
            sdk_compat.prepare_manual_control(self.drone, velocity_level=None)
            self._set(WorkerState.TAKEOFF)
            self.drone.takeoff(int(m_to_cm(cfg.speed.cruise_alt_m)))

            self._set(WorkerState.GO_TO_PAD)
            for wp in self._route_to(pad_xy, route):
                ok = fly_to_uwb(self.drone, self.uwb, self.tag_id, wp,
                                alt_m=cfg.speed.cruise_alt_m,
                                tol_m=cfg.speed.arrive_tol_m, guard=self.guard,
                                sleep=self.sleep, on_step=self._on_step, **lk)
                if not ok:
                    self.error = f"failed to reach waypoint {wp}"
                    self._set(WorkerState.FAILED)
                    return False

            self._set(WorkerState.LAND_HOOP)
            landed = land_in_hoop(
                self.drone, self.uwb, self.tag_id, pad_xy, cfg.landing.hoop_tol_m,
                footprints=self.footprints, cruise_alt_m=cfg.speed.cruise_alt_m,
                rate_hz=cfg.speed.ctrl_rate_hz, kp_xy=cfg.speed.kp_xy,
                kp_alt=cfg.speed.kp_alt, max_mps=cfg.speed.max_mps,
                climb_mps=cfg.speed.climb_mps, yaw_offset_deg=cfg.frame.yaw_offset_deg,
                invert_forward=cfg.frame.invert_forward,
                invert_right=cfg.frame.invert_right,
                hold_on_dropout=cfg.uwb.hold_on_dropout, sleep=self.sleep,
                confirm_pad=cfg.landing.confirm_pad_aruco, stream=self.stream,
                on_step=self._on_step)
            self.landed_ok = landed
            self._set(WorkerState.LANDED if landed else WorkerState.FAILED)
            return landed
        except Exception as exc:                       # one drone must never freeze others
            self.error = repr(exc)
            self._set(WorkerState.FAILED)
            try:
                self.drone.land()
            except Exception:
                pass
            return False
