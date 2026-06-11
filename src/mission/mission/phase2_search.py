"""Phase 2 — search + lock-on + tag (the ambush).

Persistent **vantage patrol / chokepoint overwatch** (cycle preplanned look-points and
re-observe lanes — NOT a one-pass lawnmower; moving targets need frequent re-observation).
`lock_and_tag` is a **visual servo** on the marker's pixel offset (station-keep over a
near-stationary target, velocity-match a mover), bounded & time-boxed, banking distinct ids
to the shared `MissionState`. Bubble-gating keeps each drone in its zone, with a
**commitment rule** (a started lock finishes across the boundary — lock_and_tag is bounded)
and a **mop-up** endgame (drop gating when only stragglers remain).

≤0.5 m/s everywhere (clamped at the stick); never overflies a footprint (routes + guard).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from mission.control.uwb_loop import fly_to_uwb
from mission.perception.aruco import confirm_with_aruco, is_rover_id
from mission.perception.detector import Detection, frame_bgr
from mission.perception.evidence import annotate as _annotate, caption_for
from mission.planner.geometry import Rect
from mission.planner.projection import CameraIntrinsics, pixel_to_arena
from mission.world.taskboard import Track

Point = Tuple[float, float]


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def point_in_poly(p: Point, poly: Sequence[Point]) -> bool:
    """Ray-cast point-in-polygon (arena n,e)."""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > p[1]) != (yj > p[1])) and \
                (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi + 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def _alt_up(drone, alt_m: float, kp_alt: float = 0.8, climb_mps: float = 0.5) -> float:
    err = alt_m - drone.get_altitude() / 100.0
    v = max(-climb_mps, min(climb_mps, kp_alt * err))
    return max(-1.0, min(1.0, v / climb_mps if climb_mps > 0 else 0.0))


def drone_arena_xy(drone, uwb=None, tag_id: Optional[int] = None) -> Optional[Point]:
    """Drone arena (north,east) from UWB (real + sim). Falls back to the fake's truth
    attrs only when UWB has no fix — NEVER assumes the real DroneAPI exposes `.n`/`.e`."""
    if uwb is not None and tag_id is not None:
        x, y, _ = uwb.get_tag_position(tag_id)
        if x is not None and y is not None:
            return (x, y)
    n, e = getattr(drone, "n", None), getattr(drone, "e", None)
    return (n, e) if n is not None and e is not None else None


def _marker_xy(bbox, cam_xy: Optional[Point], yaw_deg: float, alt_m: float,
               gimbal_deg: float, intr: Optional[CameraIntrinsics]) -> Optional[Point]:
    """Project a marker bbox centre to the floor, given the drone's UWB camera position."""
    if intr is None or cam_xy is None:
        return cam_xy
    x, y, w, h = bbox
    try:
        return pixel_to_arena(x + w / 2, y + h / 2, cam_xy, yaw_deg, alt_m, gimbal_deg, intr)
    except ValueError:
        return cam_xy


# --------------------------------------------------------------------------- #
# scan-while-transit banker  (R4) — the single banking chokepoint
# --------------------------------------------------------------------------- #
@dataclass
class ScanOutcome:
    """What one `ScanBanker.scan` frame produced."""
    banked: set = field(default_factory=set)        # ids banked on THIS frame
    in_view: set = field(default_factory=set)        # decodable rover ids in view (banked or not)
    marginal: List[Tuple[int, Detection]] = field(default_factory=list)  # in-zone, decodable,
    #                                                  NOT gate-passing → hand to lock_and_tag
    decoded_bboxes: List[Tuple[int, int, int, int]] = field(default_factory=list)  # EVERY decoded
    #                marker (rover or pad) — an "explained" blob the persist channel must skip


class ScanBanker:
    """Continuous scan-while-transit banker (R4). Run it on **every** frame — during transit
    hops AND vantage dwells. A rover id is banked the instant it clears the gate (≥`min_marker_px`,
    fully in-frame, inside this drone's zone) on `hold_frames` **consecutive** frames seen by THIS
    drone — no deliberate dwell-lock required, so a drone banks a rover it resolves while flying.

    De-dup is by id (HARD invariant #5): an **already-banked** id is ignored (its streak is dropped
    — **no re-lock, no re-bank, no dwell**). On a bank it writes annotated evidence, records the
    `drone_id` + PNG path, rebuilds the gallery and **returns the camera to the search pitch**
    (bank-and-release). It is the single chokepoint every bank routes through (scan, lock_and_tag,
    persist_and_read) so evidence + provenance + release are uniform."""

    def __init__(self, drone, uwb, tag_id: int, state, *, allow, intrinsics=None,
                 gimbal_deg: float = 90.0, search_pitch_deg: Optional[float] = None,
                 hold_frames: int = 5, min_marker_px: int = 40, frame_margin_px: int = 8,
                 dictionary: str = "DICT_6X6_250", evidence=None, taskboard=None,
                 in_zone: Optional[Callable[[Optional[Point]], bool]] = None,
                 clock: Callable[[], float] = time.time):
        self.drone = drone
        self.uwb = uwb
        self.tag_id = tag_id
        self.state = state
        self.allow = set(allow)
        self.intr = intrinsics or CameraIntrinsics()
        self.gimbal_deg = gimbal_deg
        self.search_pitch_deg = gimbal_deg if search_pitch_deg is None else search_pitch_deg
        self.hold_frames = int(hold_frames)
        self.min_marker_px = int(min_marker_px)
        self.frame_margin_px = int(frame_margin_px)
        self.dictionary = dictionary
        self.evidence = evidence
        self.taskboard = taskboard
        self.in_zone = in_zone
        self.clock = clock
        self._streak: Dict[int, int] = {}

    def _gate(self, bbox, shape) -> bool:
        """The referee gate: marker big enough AND fully in-frame (with a margin)."""
        bx, by, bw, bh = bbox
        h_px, w_px = shape[0], shape[1]
        m = self.frame_margin_px
        in_frame = (bx >= m and by >= m and bx + bw <= w_px - 1 - m
                    and by + bh <= h_px - 1 - m)
        return max(bw, bh) >= self.min_marker_px and in_frame

    def scan(self, bgr, *, cam_xy: Optional[Point] = None,
             yaw: Optional[float] = None, alt_m: Optional[float] = None) -> ScanOutcome:
        """Process one camera frame. Banks every gate-passing, in-zone, un-banked rover id whose
        consecutive-frame streak reaches `hold_frames`. Logs each sighting to the taskboard."""
        if cam_xy is None:
            cam_xy = drone_arena_xy(self.drone, self.uwb, self.tag_id)
        if yaw is None:
            yaw = self.drone.get_orientation().yaw
        if alt_m is None:
            alt_m = self.drone.get_altitude() / 100.0
        out = ScanOutcome()
        qualifying: set = set()
        for d in confirm_with_aruco(bgr, self.dictionary):
            mid = d.marker_id
            out.decoded_bboxes.append(d.bbox)        # any decoded marker is an EXPLAINED blob
            if mid is None or not is_rover_id(mid, self.allow):
                continue
            out.in_view.add(mid)
            xy = _marker_xy(d.bbox, cam_xy, yaw, alt_m, self.gimbal_deg, self.intr)
            if self.taskboard is not None:
                self.taskboard.see(Track(marker_id=mid,
                                         xy=xy if xy is not None else cam_xy, t=self.clock()))
            if self.state.is_tagged(mid):            # bank-and-release: ignore an already-banked id
                self._streak.pop(mid, None)
                continue
            if self.in_zone is not None and not self.in_zone(xy):
                self._streak.pop(mid, None)          # out of our zone (logged) — not ours to bank
                continue
            if not self._gate(d.bbox, bgr.shape):
                self._streak.pop(mid, None)          # decodable but marginal → centre via lock_and_tag
                out.marginal.append((mid, d))
                continue
            qualifying.add(mid)
            self._streak[mid] = self._streak.get(mid, 0) + 1
            if self._streak[mid] >= self.hold_frames:
                if self.bank_one(mid, d.bbox, bgr, xy):
                    out.banked.add(mid)
        for mid in list(self._streak):               # streaks must be CONSECUTIVE
            if mid not in qualifying:
                self._streak.pop(mid, None)
        return out

    def bank_one(self, marker_id: int, bbox, bgr, xy: Optional[Point]) -> bool:
        """Single bank chokepoint: de-dup, store the annotated frame, write the PNG + record the
        drone_id/path, rebuild the gallery, and return the camera to the search pitch. Returns
        True iff this id was newly banked."""
        if self.state.is_tagged(marker_id):
            return False
        t = self.clock()
        annotated = _annotate(bgr, bbox, marker_id, caption_for(self.tag_id, t, xy))
        if not self.state.bank(marker_id, annotated, xy, t, drone_id=self.tag_id):
            return False
        self._streak.pop(marker_id, None)
        if self.evidence is not None:
            path = self.evidence.write(annotated, marker_id)
            self.state.set_path(marker_id, path)
            self.evidence.gallery(self.state.evidence())
        self.drone.set_camera_angle(_DOWN, self.search_pitch_deg)   # release → resume search pitch
        return True


# --------------------------------------------------------------------------- #
# lock-on + tag (visual servo)
# --------------------------------------------------------------------------- #
def lock_and_tag(drone, stream, detection, state, *, hold_frames: int = 5,
                 center_tol_px: int = 45, min_marker_px: int = 40,
                 frame_margin_px: int = 8, lock_timeout_s: float = 6.0,
                 rate_hz: float = 20.0, kp_px: float = 0.02, gimbal_deg: float = 90.0,
                 max_mps: float = 0.5, alt_m: float = 1.1,
                 intrinsics: Optional[CameraIntrinsics] = None, uwb=None,
                 tag_id: Optional[int] = None,
                 dictionary: str = "DICT_6X6_250", sleep=time.sleep,
                 should_stop: Optional[Callable[[], bool]] = None,
                 clock: Callable[[], float] = time.time, on_step=None,
                 banker: Optional["ScanBanker"] = None) -> bool:
    """Centre `detection.marker_id` and hold `hold_frames` gate-passing frames, then bank it.
    Bounded by `lock_timeout_s` (no deadlock). Returns True iff banked.

    R4: this is now only the **centring assist** for a marginal/edge read — the continuous
    `ScanBanker` banks well-framed reads during transit/patrol without a lock. An **already-
    banked** id is a no-op (bank-and-release — no re-lock, no camera move). When a `banker` is
    given, the bank routes through it (uniform evidence + provenance + camera release)."""
    target_id = detection.marker_id
    if target_id is None or state.is_tagged(target_id):   # bank-and-release: never re-lock
        return False
    intr = intrinsics or CameraIntrinsics()
    cx, cy = intr.cx, intr.cy
    drone.set_camera_angle(_DOWN, gimbal_deg)
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05
    steps = max(1, int(lock_timeout_s * rate_hz))
    held = 0
    for _ in range(steps):
        if should_stop is not None and should_stop():
            return False
        up = _alt_up(drone, alt_m)
        frame = stream.latest_frame
        if frame is None:
            drone.send_manual_control(0.0, 0.0, up, 0.0)
            sleep(dt)
            continue
        bgr = frame_bgr(frame)
        match = next((d for d in confirm_with_aruco(bgr, dictionary)
                      if d.marker_id == target_id), None)
        if match is None:
            held = 0
            drone.send_manual_control(0.0, 0.0, up, 0.0)   # lost it → hold, keep looking
            sleep(dt)
            continue
        bx, by, bw, bh = match.bbox
        ex, ey = (bx + bw / 2) - cx, (by + bh / 2) - cy   # +ex=east, +ey=north (nadir)
        # Bank on the SAME gate the referee scores on: marker big enough AND fully in
        # frame, held `hold_frames` frames — NOT tight centering (a moving rover is rarely
        # dead-centre; the camera only needs to hold it in view).
        h_px, w_px = bgr.shape[0], bgr.shape[1]
        side = max(bw, bh)
        in_frame = (bx >= frame_margin_px and by >= frame_margin_px
                    and bx + bw <= w_px - 1 - frame_margin_px
                    and by + bh <= h_px - 1 - frame_margin_px)
        if side >= min_marker_px and in_frame:
            held += 1
            if held >= hold_frames:
                cam_xy = drone_arena_xy(drone, uwb, tag_id)
                xy = _marker_xy(match.bbox, cam_xy, drone.get_orientation().yaw,
                                drone.get_altitude() / 100.0, gimbal_deg, intr)
                if banker is not None:                      # uniform evidence + bank-and-release
                    banker.bank_one(target_id, match.bbox, bgr, xy)
                else:
                    state.bank(target_id, bgr, xy, clock())   # BGR evidence (imwrite-ready)
                drone.send_manual_control(0.0, 0.0, up, 0.0)
                return True
        else:
            held = 0
        # servo toward centre to KEEP the marker in frame (not to satisfy a centre gate)
        fwd = max(-1.0, min(1.0, kp_px * ey))
        right = max(-1.0, min(1.0, kp_px * ex))
        drone.send_manual_control(fwd, right, up, 0.0)
        if on_step is not None:
            on_step({"phase": "lock", "offset_px": math.hypot(ex, ey), "held": held})
        sleep(dt)
    return False


# --------------------------------------------------------------------------- #
# persistence on a seen-but-unread (out-of-cone) rover  (R2)
# --------------------------------------------------------------------------- #
def _clip(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if v < lo else hi if v > hi else v


def _on_footprint(xy: Point, footprints) -> bool:
    """True if arena point `xy` sits on a (raw) crate footprint — used to reject body blobs
    that are actually obstacles, not rovers."""
    for cn, ce, sn, se in footprints or ():
        if abs(xy[0] - cn) <= sn / 2 and abs(xy[1] - ce) <= se / 2:
            return True
    return False


def _bbox_overlaps_center(bbox, exclude_bboxes) -> bool:
    """True if `bbox`'s centre lies inside any excluded bbox (a decoded, EXPLAINED marker)."""
    bx, by, bw, bh = bbox
    cxp, cyp = bx + bw / 2.0, by + bh / 2.0
    for ex, ey, ew, eh in exclude_bboxes or ():
        if ex <= cxp <= ex + ew and ey <= cyp <= ey + eh:
            return True
    return False


def _nearest_presence(presence, bgr, target_xy, drone, gimbal_deg, intr, uwb, tag_id,
                      footprints, bounds, min_area_px, exclude_bboxes=()):
    """The body-blob candidate that best explains a rover near `target_xy`: projected onto the
    floor (live pitch), in bounds, NOT on a crate footprint, big enough. Returns the Detection
    (with its bbox) or None. This is the presence channel — a rover the gimbal hasn't yet shown.

    A blob whose centre sits on a DECODED marker (`exclude_bboxes`) is skipped: that rover is
    already explained (banked / out-of-zone) — persisting on it would waste the hold on an
    already-handled target instead of the genuinely-unread (out-of-cone) body."""
    cam_xy = drone_arena_xy(drone, uwb, tag_id)
    yaw = drone.get_orientation().yaw
    alt_m = drone.get_altitude() / 100.0
    best, best_d = None, float("inf")
    for c in presence.detect(bgr):
        bx, by, bw, bh = c.bbox
        if bw * bh < min_area_px:
            continue
        if _bbox_overlaps_center(c.bbox, exclude_bboxes):
            continue                                  # explained by a decoded marker → not a mystery body
        xy = _marker_xy(c.bbox, cam_xy, yaw, alt_m, gimbal_deg, intr)
        if xy is None:
            continue
        if bounds is not None and not bounds.contains(xy):
            continue
        if _on_footprint(xy, footprints):
            continue
        d = math.hypot(xy[0] - target_xy[0], xy[1] - target_xy[1]) if target_xy else 0.0
        if d < best_d:
            best, best_d = c, d
    return best


def _light_orbit(drone, uwb, tag_id, step_m, footprints, bounds, alt_m, *, guard=None,
                 sleep=time.sleep, on_step=None, **loop_kwargs) -> bool:
    """A LIGHT lateral strafe (≤ step_m, EAST or WEST) to change the rover->drone bearing when a
    hold is stuck (unlucky gimbal phase / line-of-sight blocked). Lateral only — never `+up`,
    never onto a footprint. Returns True if it moved."""
    cur = drone_arena_xy(drone, uwb, tag_id)
    if cur is None:
        return False
    for sign in (1.0, -1.0):
        tgt = (cur[0], cur[1] + sign * step_m)
        if bounds is not None and not bounds.contains(tgt):
            continue
        if _on_footprint(tgt, footprints):
            continue
        if on_step is not None:
            on_step({"phase": "orbit"})
        fly_to_uwb(drone, uwb, tag_id, tgt, alt_m=alt_m, guard=guard, sleep=sleep,
                   on_step=on_step, **loop_kwargs)
        return True
    return False


def persist_and_read(drone, stream, target_xy, state, *, allow, dictionary,
                     gimbal_deg, intrinsics, uwb, tag_id, alt_m, presence, footprints,
                     bounds, persist_timeout_s=9.0, orbit_step_m=0.6, max_orbits=2,
                     hold_frames=5, min_marker_px=40, frame_margin_px=8,
                     presence_min_area_px=500, kp_px=0.02, rate_hz=20.0, guard=None,
                     sleep=time.sleep, clock: Callable[[], float] = time.time,
                     should_stop: Optional[Callable[[], bool]] = None, on_step=None,
                     banker: Optional["ScanBanker"] = None, **loop_kwargs) -> Optional[int]:
    """Hold on a SEEN-but-UNREAD rover (body visible, marker out of the gimbal cone): keep it
    framed at the held SEARCH PITCH and wait for the sweeping gimbal to bring its marker into
    the cone, then bank it. The target stays ACTIVE across ticks — it is NOT dropped just
    because cv2.aruco can't decode this frame. After `persist_timeout_s` without a read a LIGHT
    lateral orbit changes the bearing (fallback, not the primary move). Returns the banked id,
    or None (timed out / already-banked / lost)."""
    intr = intrinsics or CameraIntrinsics()
    cx, cy = intr.cx, intr.cy
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05
    hold_steps = max(1, int(persist_timeout_s * rate_hz))
    drone.set_camera_angle(_DOWN, gimbal_deg)            # held MODERATE; never steepen to nadir
    held = 0
    for orbit in range(max_orbits + 1):
        for _ in range(hold_steps):
            if should_stop is not None and should_stop():
                return None
            up = _alt_up(drone, alt_m)
            frame = stream.latest_frame
            if frame is None:
                drone.send_manual_control(0.0, 0.0, up, 0.0)
                sleep(dt)
                continue
            bgr = frame_bgr(frame)
            decoded = confirm_with_aruco(bgr, dictionary)
            rovers = [d for d in decoded
                      if d.marker_id is not None and is_rover_id(d.marker_id, allow)]
            unbanked = [d for d in rovers if not state.is_tagged(d.marker_id)]
            if unbanked:                                  # gimbal swept a marker into the cone
                d = unbanked[0]
                bx, by, bw, bh = d.bbox
                h_px, w_px = bgr.shape[0], bgr.shape[1]
                in_frame = (bx >= frame_margin_px and by >= frame_margin_px
                            and bx + bw <= w_px - 1 - frame_margin_px
                            and by + bh <= h_px - 1 - frame_margin_px)
                if max(bw, bh) >= min_marker_px and in_frame:
                    held += 1
                    if held >= hold_frames:
                        cam_xy = drone_arena_xy(drone, uwb, tag_id)
                        xy = _marker_xy(d.bbox, cam_xy, drone.get_orientation().yaw,
                                        drone.get_altitude() / 100.0, gimbal_deg, intr)
                        if banker is not None:              # uniform evidence + bank-and-release
                            banker.bank_one(d.marker_id, d.bbox, bgr, xy)
                        else:
                            state.bank(d.marker_id, bgr, xy, clock())
                        drone.send_manual_control(0.0, 0.0, up, 0.0)
                        return d.marker_id
                else:
                    held = 0
                ex, ey = (bx + bw / 2) - cx, (by + bh / 2) - cy
                drone.send_manual_control(_clip(kp_px * ey), _clip(kp_px * ex), up, 0.0)
            else:                                         # no UNBANKED marker → keep the BODY framed
                held = 0                                  # (skip ANY decoded/banked marker's blob)
                cand = _nearest_presence(
                    presence, bgr, target_xy, drone, gimbal_deg, intr, uwb, tag_id,
                    footprints, bounds, presence_min_area_px,
                    exclude_bboxes=[d.bbox for d in decoded])
                if cand is not None:                      # still an unread body → hold on IT
                    bx, by, bw, bh = cand.bbox
                    ex, ey = (bx + bw / 2) - cx, (by + bh / 2) - cy
                    drone.send_manual_control(_clip(kp_px * ey), _clip(kp_px * ex), up, 0.0)
                elif rovers:                              # only banked rover(s), no unread body
                    return None                           # → release (bank-and-release)
                else:
                    drone.send_manual_control(0.0, 0.0, up, 0.0)   # body lost → station-keep
            if on_step is not None:
                on_step({"phase": "persist", "orbit": orbit})
            sleep(dt)
        if orbit < max_orbits:                            # held a full sweep with no read → orbit
            _light_orbit(drone, uwb, tag_id, orbit_step_m, footprints, bounds, alt_m,
                         guard=guard, sleep=sleep, on_step=on_step, **loop_kwargs)
    return None


# --------------------------------------------------------------------------- #
# vantage patrol
# --------------------------------------------------------------------------- #
def vantage_patrol(drone, uwb, tag_id: int, vantages: Sequence[dict],
                   on_dwell: Callable[[], bool], *, gimbal_deg: float = 90.0,
                   dwell_s: float = 1.0, alt_m: float = 1.1, guard=None, graph=None,
                   rate_hz: float = 20.0, sleep=time.sleep, on_step=None,
                   on_frame: Optional[Callable[[], bool]] = None, **loop_kwargs) -> bool:
    """One cycle of overwatch: fly to each vantage, tilt the gimbal, dwell while calling
    `on_dwell()` per frame. `on_dwell` returns True to stop the whole patrol (done).

    `on_frame` (R4) is the **scan-while-transit** hook: it fires every control step of each
    vantage HOP so a drone banks a gate-passing read while flying between vantages (the dwell
    keeps scanning via `on_dwell`, so each frame is scanned exactly once — no double count).

    With `graph`, each vantage hop is routed **around inflated footprints** (no-overfly is
    structural in Phase 2 too — a straight hop can cut over a crate, breaching compliance)."""
    from mission.planner.geometry import plan_path
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05

    def _transit_step(info):
        if on_step is not None:
            on_step(info)
        if on_frame is not None:
            on_frame()                       # scan-while-transit (banks mid-hop; passive, no motion)

    for v in vantages:
        target = (float(v["xy"][0]), float(v["xy"][1]))
        waypoints = [target]
        if graph is not None:
            cur = drone_arena_xy(drone, uwb, tag_id) or target
            path = plan_path(cur, target, graph)
            if path:
                waypoints = path[1:] if len(path) > 1 else path
        for wp in waypoints:
            fly_to_uwb(drone, uwb, tag_id, wp, alt_m=alt_m, guard=guard,
                       rate_hz=rate_hz, sleep=sleep, on_step=_transit_step, **loop_kwargs)
        drone.set_camera_angle(_DOWN, float(v.get("gimbal_deg", gimbal_deg)))
        dwell_steps = max(1, int(float(v.get("dwell_s", dwell_s)) * rate_hz))
        for _ in range(dwell_steps):
            if on_dwell():
                return True
            drone.send_manual_control(0.0, 0.0, _alt_up(drone, alt_m), 0.0)
            if on_step is not None:
                on_step({"phase": "dwell"})
            sleep(dt)
    return False


# --------------------------------------------------------------------------- #
# the per-drone phase-2 driver
# --------------------------------------------------------------------------- #
def phase2_search(drone, uwb, tag_id: int, vantages: Sequence[dict], stream, state,
                  taskboard, *, bubble: Optional[Sequence[Point]] = None,
                  all_ids: Optional[Sequence[int]] = None, guard=None,
                  budget_cycles: int = 6, mopup_extra_cycles: int = 2,
                  dwell_s: float = 1.0, gimbal_deg: float = 90.0, rate_hz: float = 20.0,
                  lock_timeout_s: float = 6.0, center_tol_px: int = 45,
                  min_marker_px: int = 40, kp_px: float = 0.02, hold_frames: int = 5,
                  intrinsics: Optional[CameraIntrinsics] = None, alt_m: float = 1.1,
                  rover_ids: Optional[Sequence[int]] = None,
                  dictionary: str = "DICT_6X6_250",
                  presence=None, footprints: Sequence = (), bounds=None,
                  persist_timeout_s: float = 9.0, orbit_step_m: float = 0.6,
                  max_orbits: int = 2, presence_min_area_px: int = 500,
                  release_radius_m: float = 0.8,
                  phase_budget_s: Optional[float] = None,
                  now: Callable[[], float] = time.monotonic,
                  evidence=None, graph=None, sleep=time.sleep,
                  clock: Callable[[], float] = time.time,
                  on_step=None, **loop_kwargs) -> set:
    """Patrol vantages and bank distinct rover ids within `bubble` (mop-up drops the gate near
    the end). Banks ONLY ids in the `rover_ids` allow-list (defaults to `all_ids`).

    R4: a `ScanBanker` runs on **every** frame — during the vantage HOPS (scan-while-transit)
    AND the dwells — so a drone banks a gate-passing read while flying, without a deliberate
    dwell-lock, and **releases** an id the instant it banks (already-banked → ignored). Every
    bank writes annotated evidence (R4c) when an `evidence` writer is given. `lock_and_tag` is
    kept only to **centre a marginal/edge read** until it clears the gate.

    R2 (when `presence` — a body detector — is given): a rover whose marker is OUT of the gimbal
    cone shows only its body. The drone PERSISTS on that body (holding the search pitch, keeping
    it framed) until the sweeping gimbal brings the marker into the cone and it banks; a light
    lateral orbit breaks a stuck hold. Returns the set of ids THIS drone banked."""
    banked: set = set()
    flags = {"mopup": False}
    released: List[Point] = []                              # bodies persist gave up on (never re-hold)
    allow = set(rover_ids if rover_ids is not None else (all_ids or []))
    t_start = now()

    def done() -> bool:
        return all_ids is not None and len(set(all_ids) - state.tagged()) == 0

    def expired() -> bool:
        """Hard Phase-2 wall-clock cap: the mission ALWAYS terminates (then lands all in
        `finally`) even if a rover is permanently out of cone / out of read range."""
        return phase_budget_s is not None and (now() - t_start) >= phase_budget_s

    def stop() -> bool:
        return done() or expired()

    def _is_released(xy) -> bool:
        """A body persist already gave up on (out of read range) — don't re-hold; move on."""
        return xy is not None and any(
            math.hypot(xy[0] - r[0], xy[1] - r[1]) <= release_radius_m for r in released)

    def _in_zone(xy) -> bool:
        return (bubble is None or flags["mopup"]
                or (xy is not None and point_in_poly(xy, bubble)))

    banker = ScanBanker(drone, uwb, tag_id, state, allow=allow, intrinsics=intrinsics,
                        gimbal_deg=gimbal_deg, search_pitch_deg=gimbal_deg,
                        hold_frames=hold_frames, min_marker_px=min_marker_px,
                        dictionary=dictionary, evidence=evidence, taskboard=taskboard,
                        in_zone=_in_zone, clock=clock)

    def transit_scan() -> bool:
        """on_frame: scan-while-transit — bank a gate-passing read mid-hop (passive, no motion)."""
        if stop():
            return True
        frame = stream.latest_frame
        if frame is None:
            return False
        banked.update(banker.scan(frame_bgr(frame)).banked)
        return stop()

    def scan_and_lock() -> bool:
        if stop():
            return True
        frame = stream.latest_frame
        if frame is None:
            return False
        bgr = frame_bgr(frame)
        out = banker.scan(bgr)                             # 1) continuous gate (transit + dwell)
        banked.update(out.banked)
        if stop():
            return True
        if out.banked:                                     # handled an unbanked in-zone read
            return False                                   #   this frame → don't also persist
        # 2) centre a MARGINAL in-zone read (edge/too-small) so it clears the gate — the only
        #    place a deliberate lock happens now; well-framed reads bank via the gate above.
        if out.marginal:
            mid, det = out.marginal[0]
            if not state.is_tagged(mid) and lock_and_tag(
                    drone, stream, det, state, hold_frames=hold_frames,
                    center_tol_px=center_tol_px, min_marker_px=min_marker_px,
                    lock_timeout_s=lock_timeout_s, dictionary=dictionary, rate_hz=rate_hz,
                    kp_px=kp_px, gimbal_deg=gimbal_deg, intrinsics=intrinsics, uwb=uwb,
                    tag_id=tag_id, alt_m=alt_m, sleep=sleep, clock=clock, on_step=on_step,
                    should_stop=stop, banker=banker):
                banked.add(mid)
            return stop()
        # 3) R2 — reaching here means NO unbanked in-zone decodable rover to act on (banked
        #    + marginal both empty). If a body is present (its marker out of the gimbal cone),
        #    PERSIST until the sweep brings the marker in. NOT gated on `in_view`: an ALREADY-
        #    BANKED marker sharing the frame must not block persisting on a DIFFERENT out-of-
        #    cone rover (that gate regressed R2's persistence — convoy id 67 went unbanked).
        if presence is not None:
            cam_xy = drone_arena_xy(drone, uwb, tag_id)
            yaw, alt = drone.get_orientation().yaw, drone.get_altitude() / 100.0
            cand = _nearest_presence(presence, bgr, None, drone, gimbal_deg, intrinsics,
                                     uwb, tag_id, footprints, bounds, presence_min_area_px,
                                     exclude_bboxes=out.decoded_bboxes)   # skip banked-marker blobs
            if cand is not None:
                xy = _marker_xy(cand.bbox, cam_xy, yaw, alt, gimbal_deg, intrinsics)
                if _in_zone(xy) and not _is_released(xy):   # released → out of read range, move on
                    taskboard.see(Track(marker_id=None, xy=xy if xy else cam_xy, t=clock()))
                    mid = persist_and_read(
                        drone, stream, xy, state, allow=allow, dictionary=dictionary,
                        gimbal_deg=gimbal_deg, intrinsics=intrinsics, uwb=uwb, tag_id=tag_id,
                        alt_m=alt_m, presence=presence, footprints=footprints, bounds=bounds,
                        persist_timeout_s=persist_timeout_s, orbit_step_m=orbit_step_m,
                        max_orbits=max_orbits, hold_frames=hold_frames,
                        min_marker_px=min_marker_px, presence_min_area_px=presence_min_area_px,
                        kp_px=kp_px, rate_hz=rate_hz, guard=guard, sleep=sleep, clock=clock,
                        should_stop=stop, on_step=on_step, banker=banker)
                    if mid is not None:
                        banked.add(mid)
                    elif xy is not None and not expired():   # timed out + orbited, still no read →
                        released.append(xy)                  #   RELEASE this rover, never re-hold it
                    return stop()
        return stop()

    for cycle in range(budget_cycles):
        if stop():                                        # done OR Phase-2 wall-clock cap hit
            break
        if all_ids is not None and cycle >= budget_cycles - mopup_extra_cycles:
            flags["mopup"] = True                         # endgame: drop the gate
        if vantage_patrol(drone, uwb, tag_id, vantages, scan_and_lock,
                          gimbal_deg=gimbal_deg, dwell_s=dwell_s, alt_m=alt_m,
                          guard=guard, graph=graph, rate_hz=rate_hz, sleep=sleep,
                          on_step=on_step, on_frame=transit_scan, **loop_kwargs):
            break
    return banked


# --------------------------------------------------------------------------- #
# P8 — adversarial evader handling
# --------------------------------------------------------------------------- #
def classify_behaviour(samples: Sequence[Point], *, erratic_turn_std_deg: float = 45.0,
                       periodic_close_frac: float = 0.25,
                       min_path_m: float = 0.3) -> str:
    """Triage a track's recent path → 'smooth' | 'periodic' | 'erratic' | 'unknown'.

    Erratic (human evader) ⇒ high turn-angle variance. Periodic (loop) ⇒ low variance and
    returns near its start. Smooth (autonomous convoy) ⇒ low variance, open path."""
    pts = [(float(p[0]), float(p[1])) for p in samples]
    if len(pts) < 3:
        return "unknown"
    turns: List[float] = []
    path_len = 0.0
    for i in range(1, len(pts) - 1):
        v1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        v2 = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        path_len += math.hypot(*v1)
        if math.hypot(*v1) < 1e-9 or math.hypot(*v2) < 1e-9:
            continue
        ang = math.degrees(math.atan2(v1[0] * v2[1] - v1[1] * v2[0],
                                      v1[0] * v2[0] + v1[1] * v2[1]))
        turns.append(ang)
    path_len += math.hypot(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1])
    if not turns or path_len < min_path_m:
        return "unknown"
    std = float(np.std(turns))
    if std > erratic_turn_std_deg:
        return "erratic"
    closes = math.dist(pts[0], pts[-1]) < periodic_close_frac * path_len
    return "periodic" if closes else "smooth"


class ReachableSet:
    """Where an evader could be, on the free-space grid. Holding a chokepoint `cut`s the
    set; with no expansion the size is monotonically non-increasing → containment shrinks
    the search (docs/ARCHITECTURE.md). Lane-graph reachability, not probability."""

    def __init__(self, bounds: Rect,
                 footprints: Sequence[Tuple[float, float, float, float]] = (),
                 cell_size: float = 0.25):
        self.bounds = bounds
        self.cell = float(cell_size)
        self.nn = max(1, int(round((bounds.max_n - bounds.min_n) / self.cell)))
        self.ne = max(1, int(round((bounds.max_e - bounds.min_e) / self.cell)))
        self.free = np.ones((self.nn, self.ne), dtype=bool)
        for i in range(self.nn):
            for j in range(self.ne):
                cn, ce = self._center(i, j)
                for fn, fe, sn, se in footprints:
                    if abs(cn - fn) <= sn / 2 and abs(ce - fe) <= se / 2:
                        self.free[i, j] = False
                        break
        self.barrier = np.zeros((self.nn, self.ne), dtype=bool)
        self.reach = self.free.copy()
        self._anchor: Optional[Tuple[int, int]] = None

    def _center(self, i: int, j: int) -> Point:
        return (self.bounds.min_n + (i + 0.5) * self.cell,
                self.bounds.min_e + (j + 0.5) * self.cell)

    def cell_of(self, xy: Point) -> Tuple[int, int]:
        i = int((xy[0] - self.bounds.min_n) / self.cell)
        j = int((xy[1] - self.bounds.min_e) / self.cell)
        return (min(max(i, 0), self.nn - 1), min(max(j, 0), self.ne - 1))

    def _component(self, start: Tuple[int, int]) -> np.ndarray:
        from collections import deque
        passable = self.free & ~self.barrier
        out = np.zeros_like(self.free)
        if not passable[start]:
            return out
        out[start] = True
        q = deque([start])
        while q:
            i, j = q.popleft()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if 0 <= ni < self.nn and 0 <= nj < self.ne and \
                        passable[ni, nj] and not out[ni, nj]:
                    out[ni, nj] = True
                    q.append((ni, nj))
        return out

    def seed(self, xy: Point) -> None:
        self._anchor = self.cell_of(xy)
        self.reach = self._component(self._anchor)

    def cut(self, xy: Point, radius_m: float = 0.4) -> None:
        ci, cj = self.cell_of(xy)
        r = int(math.ceil(radius_m / self.cell))
        for i in range(max(0, ci - r), min(self.nn, ci + r + 1)):
            for j in range(max(0, cj - r), min(self.ne, cj + r + 1)):
                if math.hypot(*[a - b for a, b in zip(self._center(i, j), xy)]) <= radius_m:
                    self.barrier[i, j] = True
        if self._anchor is not None:
            self.reach = self._component(self._anchor)

    def expand(self, dt: float, speed: float) -> None:
        steps = max(1, int(round(speed * dt / self.cell)))
        passable = self.free & ~self.barrier
        for _ in range(steps):
            grown = self.reach.copy()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                grown |= np.roll(self.reach, (di, dj), axis=(0, 1))
            self.reach = grown & passable

    def size(self) -> int:
        return int(self.reach.sum())

    def contains(self, xy: Point) -> bool:
        return bool(self.reach[self.cell_of(xy)])


def plan_containment(reachable: ReachableSet, chokepoints: Sequence[Point],
                     n_blockers: int) -> List[Point]:
    """Pick up to `n_blockers` chokepoints that best bound the reachable set (those whose
    neighbourhood overlaps the reachable region most) — hold these to shrink it."""
    scored = []
    for cp in chokepoints:
        cells = 0
        ci, cj = reachable.cell_of(cp)
        r = 2
        for i in range(max(0, ci - r), min(reachable.nn, ci + r + 1)):
            for j in range(max(0, cj - r), min(reachable.ne, cj + r + 1)):
                if reachable.reach[i, j]:
                    cells += 1
        if cells > 0:
            scored.append((cells, cp))
    scored.sort(key=lambda s: -s[0])
    return [cp for _, cp in scored[:n_blockers]]


# resolved once (the fake/real enum)
def _down_mode():
    from pyhulax.core import CameraPitchMode
    return CameraPitchMode.DOWN_ABSOLUTE


_DOWN = _down_mode()
