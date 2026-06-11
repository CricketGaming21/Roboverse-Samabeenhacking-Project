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
from mission.perception.video import wait_for_first_frame
from mission.planner.geometry import plan_path
from mission.runtime import sdk_compat

Point = Tuple[float, float]


class FailsafeAbort(Exception):
    """Raised mid-flight to trigger a safe return-and-land (battery, etc.)."""

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(reason)


class WorkerState(str, Enum):
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    GO_TO_PAD = "GO_TO_PAD"
    LAND_HOOP = "LAND_HOOP"
    LANDED = "LANDED"
    # phase 2
    RELAUNCH = "RELAUNCH"
    SEARCH = "SEARCH"
    CONVERGE = "CONVERGE"
    HOME = "HOME"
    DONE = "DONE"
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
                 priority: int = 0, sleep=time.sleep,
                 battery_rtl_pct: Optional[int] = None):
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
        self.battery_rtl_pct = battery_rtl_pct
        self.state = WorkerState.INIT
        self.history: List[WorkerState] = [WorkerState.INIT]
        self.trace: List[Point] = []
        self.landed_ok = False
        self.rtl = False
        self.error: Optional[str] = None

    # -- helpers ---------------------------------------------------------- #
    def _set(self, s: WorkerState) -> None:
        self.state = s
        self.history.append(s)

    def _drone_xy(self) -> Optional[Point]:
        """Arena (north,east) from UWB (real + sim). Falls back to the fake's truth attrs
        only if UWB has no fix — NEVER assumes the real DroneAPI exposes `.n`/`.e`."""
        x, y, _ = self.uwb.get_tag_position(self.tag_id)
        if x is not None and y is not None:
            return (x, y)
        n, e = getattr(self.drone, "n", None), getattr(self.drone, "e", None)
        return (n, e) if n is not None and e is not None else None

    def _on_step(self, info: dict) -> None:
        xy = self._drone_xy()
        if xy is not None:
            self.trace.append(xy)
        if self.battery_rtl_pct is not None and \
                self.drone.get_battery() <= self.battery_rtl_pct:
            raise FailsafeAbort("battery_rtl")        # safe return-and-land

    def _current_xy(self) -> Point:
        return self._drone_xy() or (0.0, 0.0)

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
            sdk_compat.prepare_manual_control(self.drone, velocity_level=cfg.real.velocity_level,
                                              heartbeat_hz=cfg.real.heartbeat_hz)
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
                on_step=self._on_step)               # R1: UWB-only, no ArUco, camera off
            self.landed_ok = landed
            self._set(WorkerState.LANDED if landed else WorkerState.FAILED)
            return landed
        except FailsafeAbort as fa:                     # battery/etc → safe return-and-land
            self.error = fa.reason
            self.rtl = True
            self._set(WorkerState.LANDED)
            try:
                self.drone.land()
            except Exception:
                pass
            return self.landed_ok
        except Exception as exc:                       # one drone must never freeze others
            self.error = repr(exc)
            self._set(WorkerState.FAILED)
            try:
                self.drone.land()
            except Exception:
                pass
            return False

    # -- phase 2 ---------------------------------------------------------- #
    def run_phase2(self, vantages, stream, state, taskboard, *, bubble=None,
                   all_ids=None, home_xy=None, **kwargs) -> set:
        """RELAUNCH (re-takeoff if landed) → SEARCH (vantage patrol + lock/tag) →
        HOME. Returns the set of ids this drone banked."""
        from mission.mission.phase2_search import phase2_search

        cfg = self.cfg
        lk = _loop_kwargs(cfg)
        banked: set = set()
        try:
            self._set(WorkerState.RELAUNCH)
            if self.drone.get_altitude() < 30.0:           # was landed after phase 1
                sdk_compat.prepare_manual_control(self.drone, velocity_level=cfg.real.velocity_level,
                                                  heartbeat_hz=cfg.real.heartbeat_hz)
                self.drone.takeoff(int(m_to_cm(cfg.speed.cruise_alt_m)))
            if stream is not None:                         # camera ON at Phase-2 start
                sdk_compat.start_video_stream(self.drone)   # real: LOW res first (3 streams);
                stream.start()                              # (off during Phase-1 UWB landing)
                # Real H.264 negotiation can take several seconds; wait for the first frame so
                # search doesn't treat the warm-up gap as "no rovers seen" (no-op on the sim).
                wait_for_first_frame(stream, timeout_s=cfg.camera.video_warmup_timeout_s,
                                     sleep=self.sleep)

            self._set(WorkerState.SEARCH)
            # Hard Phase-2 wall-clock cap from config (overridable) so the search ALWAYS
            # terminates — a rover permanently out of cone / out of read range can never hang it.
            kwargs.setdefault("phase_budget_s", cfg.failsafe.phase2_budget_s)
            banked = phase2_search(
                self.drone, self.uwb, self.tag_id, vantages, stream, state, taskboard,
                bubble=bubble, all_ids=all_ids, guard=self.guard,
                alt_m=cfg.speed.cruise_alt_m, lock_timeout_s=cfg.failsafe.lock_timeout_s,
                sleep=self.sleep, on_step=self._on_step, **lk, **kwargs)

            if home_xy is not None:
                self._set(WorkerState.HOME)
                fly_to_uwb(self.drone, self.uwb, self.tag_id, home_xy,
                           alt_m=cfg.speed.cruise_alt_m, tol_m=cfg.speed.arrive_tol_m,
                           guard=self.guard, sleep=self.sleep,
                           on_step=self._on_step, **lk)
            self._set(WorkerState.DONE)
            return banked
        except FailsafeAbort as fa:                     # battery/etc → safe return-and-land
            self.error = fa.reason
            self.rtl = True
            self._set(WorkerState.LANDED)
            try:
                self.drone.land()
            except Exception:
                pass
            return banked
        except Exception as exc:
            self.error = repr(exc)
            self._set(WorkerState.FAILED)
            try:
                self.drone.land()
            except Exception:
                pass
            return banked
