"""READ-ONLY debug / introspection over the live sim world.

Simulator INTERNAL — for TESTS and DEBUG SCRIPTS only. Mission code must
never import this (and it is not reachable through pyhulax).

DebugProbe never mutates anything: every accessor only reads registry /
world / referee / monitor state (the referee diagnostics render a frame
through the existing camera path, which changes no state). World state is
gathered in ONE sim-thread call, so each snapshot is atomic at a step
boundary; referee/monitor counters are sampled immediately after it.

Zero overhead when unused: nothing here runs until a test or script
constructs a DebugProbe (or a --debug/--dump flag calls start_debug_loop).
"""

import json
import math
import threading
import time

import cv2
import numpy as np
import pybullet as p

from . import aruco_assets, frames, sensors


def _round3(seq):
    return [round(float(v), 3) for v in seq]


def _jsonable(obj):
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    return str(obj)


class DebugProbe:
    """Read-only window into one sim world (pass the SimRegistry)."""

    def __init__(self, registry):
        self._reg = registry
        self._cfg = registry.config
        self._detector = cv2.aruco.ArucoDetector(
            aruco_assets.get_dictionary(self._cfg.aruco.dictionary),
            cv2.aruco.DetectorParameters())

    # ------------------------------------------------------------------ #
    # Builders — SIM THREAD ONLY (called inside the snapshot closure)
    # ------------------------------------------------------------------ #

    def _goal_dict(self, drone):
        g = drone.goal
        if g is None:
            return None
        return {
            "kind": g.kind,
            "target_world": _round3(g.target_pos) if g.target_pos is not None
            else None,
            "target_yaw_rad": round(float(g.target_yaw), 4)
            if g.target_yaw is not None else None,
            "speed_mps": g.speed_mps,
            "end_time": g.end_time,
            "deadline": round(g.deadline, 2),
        }

    def _sensor_dict(self, drone):
        """The 5 barrier rays (endpoints + hit) and ToF altitude — built from
        the SAME geometry helper sensing uses (sensors.barrier_rays)."""
        cfg, client = self._cfg, self._reg.client
        alt_cm = sensors.altitude_cm(client, cfg, drone)
        gated = alt_cm / 100.0 < cfg.barrier_sensors.min_altitude_m
        rays = sensors.barrier_rays(cfg, drone)
        hits = p.rayTestBatch([r[1] for r in rays], [r[2] for r in rays],
                              physicsClientId=client)
        per = {}
        for (name, start, end), hit in zip(rays, hits):
            raw = bool(hit[0] >= 0 and hit[0] != drone.body_id)
            per[name] = {
                "blocked": bool(raw and not gated),  # == barrier_flags result
                "raw_hit": raw,
                "start": _round3(start),
                "end": _round3(end),
                "hit_pos": _round3(hit[3]) if raw else None,
            }
        return {"altitude_cm": round(float(alt_cm), 1), "gated": gated,
                "rays": per}

    def _blocked(self, drone) -> bool:
        """Is the active goal's direction of travel currently barred?"""
        g = drone.goal
        if g is None or g.target_pos is None:
            return False
        delta = g.target_pos - drone.pos
        dist = float(np.linalg.norm(delta))
        if dist < 1e-9:
            return False
        flags = drone.sense_obstacles()
        return bool(flags.any and drone._motion_blocked(delta / dist, flags))

    def _drone_dict(self, drone):
        fr = drone.takeoff_frame
        est_world = drone.pos + drone.drift_err
        est = drone.telemetry_position()
        ori = drone.telemetry_orientation()
        rule = drone.avoidance_rule
        return {
            "index": drone.index,
            "ip": drone.spec.ip,
            "connected": drone.connected,
            "flying": drone.flying,
            "battery_pct": round(float(drone.battery_pct), 2),
            "camera_pitch_deg": drone.camera_pitch_deg,
            "barrier_mode": drone.barrier_mode,
            "avoidance_rule": None if rule is None else {
                "direction": rule[0].name, "distance_cm": rule[1] * 100.0,
                "mask": int(rule[2])},
            # The same pose in every frame at once (frames.py is the truth):
            "true": {
                "world": _round3(drone.pos),
                "yaw_rad": round(float(drone.yaw), 4),
                "arena_ne_m": _round3(drone.arena_position()),
                "takeoff_cm": _round3(
                    frames.world_to_takeoff_cm(fr, *drone.pos))
                if fr is not None else None,
            },
            "estimate": {  # what get_position() reports — drifts
                "takeoff_cm": _round3((est.x, est.y, est.z)),
                "world": _round3(est_world),
                "drift_error_m": round(
                    float(np.linalg.norm(drone.drift_err)), 4),
            },
            "orientation_deg": {"yaw": round(ori.yaw, 2),
                                "pitch": ori.pitch, "roll": ori.roll},
            "goal": self._goal_dict(drone),
            "executing": drone.goal is not None,
            "blocked": self._blocked(drone),
            "sensors": self._sensor_dict(drone),
        }

    def _rover_dict(self, rover):
        wp = None
        if rover._target_w is not None:
            wp = _round3(frames.world_to_arena(
                self._cfg, rover._target_w[0], rover._target_w[1]))
        return {
            "index": rover.index,
            "marker_id": rover.marker_id,
            "arena_ne_m": _round3(rover.arena_position()),
            "world_xy": _round3(rover.pos[:2]),
            "yaw_rad": round(float(rover.yaw), 4),
            "waypoint_arena_ne_m": wp,   # None while paused
            "pause_until": round(rover._pause_until, 2) if wp is None
            else None,
        }

    # ------------------------------------------------------------------ #
    # Referee + monitor views (any thread)
    # ------------------------------------------------------------------ #

    def referee_view(self) -> dict:
        """Per visible marker id, WHY it is or isn't scoring right now:
        pixel size vs min_marker_px, fully-in-frame vs frame_margin_px,
        current hold count vs hold_frames, banked-yet."""
        ref = self._reg.referee
        cfg = self._cfg
        out = {"enabled": ref is not None}
        banked = {}
        if ref is not None:
            out["mode"] = cfg.scoring.mode
            out["score"] = ref.score()
            banked = {b.marker_id: b for b in ref.banked()}
            out["banked"] = {mid: {"drone": b.drone_index,
                                   "sim_time": round(b.sim_time, 2)}
                             for mid, b in banked.items()}
        per_drone = {}
        w, h = cfg.camera.width, cfg.camera.height
        margin, min_px = cfg.scoring.frame_margin_px, cfg.scoring.min_marker_px
        for i, drone in enumerate(self._reg.drones):
            if not (drone.connected and drone.flying):
                continue
            try:
                rgb = self._reg.render_camera(drone)
            except (RuntimeError, TimeoutError):
                break
            corners, ids, _ = self._detector.detectMarkers(
                cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
            seen = {}
            if ids is not None:
                for mid, quad in zip(ids.flatten(), corners):
                    pts = quad[0]
                    mid = int(mid)
                    side = max(float(np.linalg.norm(pts[k] - pts[(k + 1) % 4]))
                               for k in range(4))
                    in_frame = bool(
                        pts[:, 0].min() >= margin
                        and pts[:, 0].max() <= w - 1 - margin
                        and pts[:, 1].min() >= margin
                        and pts[:, 1].max() <= h - 1 - margin)
                    hold = 0
                    if ref is not None:
                        with ref._lock:
                            hold = ref._holds.get(i, {}).get(mid, 0)
                    seen[mid] = {
                        "side_px": round(side, 1),
                        "size_ok": side >= min_px,
                        "fully_in_frame": in_frame,
                        "gate_ok": side >= min_px and in_frame,
                        "hold": hold,
                        "hold_needed": cfg.scoring.hold_frames,
                        "banked": mid in banked,
                    }
            per_drone[i] = seen
        out["per_drone"] = per_drone
        return out

    def monitor_state(self) -> dict:
        mon = self._reg.monitor
        snap = mon.snapshot()
        recent, rates = {}, {}
        for i in range(len(self._reg.drones)):
            cmds = mon.recent_commands(i)
            recent[i] = [[round(t, 3), kind] for t, kind in cmds]
            if len(cmds) >= 2 and cmds[-1][0] > cmds[0][0]:
                rates[i] = round(
                    (len(cmds) - 1) / (cmds[-1][0] - cmds[0][0]), 2)
            else:
                rates[i] = 0.0
        snap["recent_commands"] = recent
        snap["recent_rate_hz"] = rates
        return snap

    # ------------------------------------------------------------------ #
    # Snapshots
    # ------------------------------------------------------------------ #

    def snapshot(self, referee_view: bool = True) -> dict:
        """Everything, as a plain dict, for the current sim tick.

        Drone/rover/sensor state is one atomic sim-thread read; the referee
        diagnostics (which render frames) and monitor counters are sampled
        immediately after it.
        """
        reg = self._reg

        def _core():
            return {
                "sim_time": round(reg.clock.now(), 4),
                "drones": [self._drone_dict(d) for d in reg.drones],
                "rovers": [self._rover_dict(r) for r in reg.rovers],
            }
        snap = reg.run_on_sim_thread(_core)
        snap["referee"] = (self.referee_view() if referee_view
                           else {"enabled": reg.referee is not None})
        snap["monitor"] = self.monitor_state()
        return snap

    def to_json(self, snap: dict = None, indent=None) -> str:
        """JSON for logging/diffing (builds a fresh snapshot if none given)."""
        return json.dumps(snap if snap is not None else self.snapshot(),
                          default=_jsonable, indent=indent)

    def format_text(self, snap: dict = None) -> str:
        """Compact human-readable digest of a snapshot."""
        if snap is None:
            snap = self.snapshot()
        ref = snap["referee"]
        head = f"[t={snap['sim_time']:7.2f}s]"
        if ref.get("enabled"):
            head += (f" score={ref['score']}"
                     f" banked={sorted(ref.get('banked', {}))}")
        lines = [head]
        for d in snap["drones"]:
            n, e = d["true"]["arena_ne_m"]
            rays = d["sensors"]["rays"]
            sens = "".join(k[0].upper() if rays[k]["blocked"] else "-"
                           for k in ("forward", "back", "left", "right",
                                     "down"))
            goal = d["goal"]["kind"] if d["goal"] else "idle"
            lines.append(
                f"  d{d['index']} {'fly' if d['flying'] else 'gnd'} "
                f"arena({n:5.2f},{e:5.2f}) alt={d['sensors']['altitude_cm']:4.0f}cm "
                f"batt={d['battery_pct']:5.1f}% goal={goal:<9s} "
                f"drift={d['estimate']['drift_error_m'] * 100:4.1f}cm "
                f"sens={sens}{' BLOCKED' if d['blocked'] else ''}")
            for mid, info in ref.get("per_drone", {}).get(
                    d["index"], {}).items():
                lines.append(
                    f"      sees id{mid}: {info['side_px']:.0f}px "
                    f"{'in' if info['fully_in_frame'] else 'NOT-in'}-frame "
                    f"hold {info['hold']}/{info['hold_needed']}"
                    f"{' BANKED' if info['banked'] else ''}")
        for r in snap["rovers"]:
            n, e = r["arena_ne_m"]
            wp = r["waypoint_arena_ne_m"]
            tail = (f" -> wp({wp[0]:.2f},{wp[1]:.2f})" if wp else " (paused)")
            lines.append(f"  r{r['index']} id{r['marker_id']} "
                         f"({n:.2f},{e:.2f}){tail}")
        return "\n".join(lines)


def start_debug_loop(registry, print_text: bool = False,
                     dump_path: str = None, period_sim_s: float = 1.0):
    """Periodically snapshot the world (sim-time paced): print a digest
    (--debug) and/or append one JSON snapshot per tick to dump_path (--dump,
    JSON Lines). Returns a zero-arg stop function. Costs nothing unless
    called."""
    probe = DebugProbe(registry)
    stop = threading.Event()
    fh = open(dump_path, "w") if dump_path else None

    def _loop():
        next_due = registry.sim_time()
        try:
            while not stop.is_set() and registry.is_alive():
                now = registry.sim_time()
                if now < next_due:
                    time.sleep(0.02)
                    continue
                try:
                    snap = probe.snapshot()
                except (RuntimeError, TimeoutError):
                    break
                if print_text:
                    print(probe.format_text(snap), flush=True)
                if fh is not None:
                    fh.write(probe.to_json(snap) + "\n")
                    fh.flush()
                next_due += period_sim_s
                if next_due < now:
                    next_due = now + period_sim_s
        finally:
            if fh is not None:
                fh.close()

    thread = threading.Thread(target=_loop, name="hula-debug", daemon=True)
    thread.start()

    def _stop():
        stop.set()
        thread.join(timeout=5)
    return _stop
