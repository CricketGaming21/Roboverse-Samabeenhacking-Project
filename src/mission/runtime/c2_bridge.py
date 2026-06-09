"""C2 bridge — serves the live mission state as JSON for the operator console (c2/index.html)
and routes operator override commands into the Coordinator.

Reads ONLY the public world model (MissionState/TaskBoard/BeliefGrid) + drone telemetry the
caller assembles — no simcore. Raises nothing on read; alarms (low battery, UWB dropout,
near-collision, **drone over a crate footprint = no-fly violation**) are surfaced as data.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from mission.world.taskboard import Assignment, Role

Point = Tuple[float, float]


class C2Bridge:
    def __init__(self, state, taskboard, belief=None, coordinator=None, *,
                 footprints: Sequence[Tuple[float, float, float, float]] = (),
                 zones: Optional[Sequence[Sequence[Point]]] = None,
                 all_ids: Sequence[int] = (), home_xy: Point = (0.5, 0.5),
                 battery_warn_pct: int = 20, near_collision_m: float = 0.6):
        self.state = state
        self.taskboard = taskboard
        self.belief = belief
        self.coordinator = coordinator
        self.footprints = [tuple(f) for f in footprints]
        self.zones = [list(z) for z in (zones or [])]
        self.all_ids = list(all_ids)
        self.home_xy = tuple(home_xy)
        self.battery_warn_pct = int(battery_warn_pct)
        self.near_collision_m = float(near_collision_m)

    # -- alarms ----------------------------------------------------------- #
    def over_footprint(self, xy: Point) -> bool:
        """No-fly violation: a drone whose (x,y) is inside a crate footprint — at ANY
        height this voids scores (HARD invariant #3)."""
        for cn, ce, sn, se in self.footprints:
            if abs(xy[0] - cn) <= sn / 2 and abs(xy[1] - ce) <= se / 2:
                return True
        return False

    def alarms(self, drones: Sequence[dict]) -> List[dict]:
        out: List[dict] = []
        for d in drones:
            xy = tuple(d["xy"])
            if self.over_footprint(xy):
                out.append({"type": "over_footprint", "tag_id": d["tag_id"],
                            "severity": "critical",
                            "msg": f"drone {d['tag_id']} OVER a crate footprint — NO-FLY"})
            if d.get("battery_pct") is not None and d["battery_pct"] <= self.battery_warn_pct:
                out.append({"type": "low_battery", "tag_id": d["tag_id"],
                            "severity": "warn", "msg": f"drone {d['tag_id']} battery "
                            f"{d['battery_pct']}%"})
            if not d.get("uwb_ok", True):
                out.append({"type": "uwb_dropout", "tag_id": d["tag_id"],
                            "severity": "warn", "msg": f"drone {d['tag_id']} UWB dropout"})
        for i in range(len(drones)):
            for j in range(i + 1, len(drones)):
                a, b = tuple(drones[i]["xy"]), tuple(drones[j]["xy"])
                if (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 < self.near_collision_m ** 2:
                    out.append({"type": "near_collision",
                                "tag_id": [drones[i]["tag_id"], drones[j]["tag_id"]],
                                "severity": "warn",
                                "msg": f"drones {drones[i]['tag_id']} & {drones[j]['tag_id']} too close"})
        return out

    # -- scoreboard / heatmap -------------------------------------------- #
    def scoreboard(self, drones: Sequence[dict]) -> dict:
        tagged = sorted(self.state.tagged())
        return {"landings": sum(1 for d in drones if d.get("state") == "LANDED"),
                "tagged_ids": tagged, "tagged_count": len(tagged),
                "outstanding_ids": sorted(set(self.all_ids) - set(tagged))}

    def _heatmap(self, max_rows: int = 16, max_cols: int = 24) -> List[List[float]]:
        if self.belief is None:
            return []
        g = self.belief.snapshot()
        rb = max(1, g.shape[0] // max_rows)
        cb = max(1, g.shape[1] // max_cols)
        rows = []
        for i in range(0, g.shape[0], rb):
            row = []
            for j in range(0, g.shape[1], cb):
                row.append(float(g[i:i + rb, j:j + cb].max()))
            rows.append(row)
        return rows

    # -- the snapshot ----------------------------------------------------- #
    def snapshot(self, drones: Sequence[dict], clock: float = 0.0) -> dict:
        return {
            "clock": float(clock),
            "drones": [{"tag_id": d["tag_id"], "xy": [float(d["xy"][0]), float(d["xy"][1])],
                        "alt_m": d.get("alt_m"), "battery_pct": d.get("battery_pct"),
                        "state": d.get("state"), "uwb_ok": d.get("uwb_ok", True),
                        "over_footprint": self.over_footprint(tuple(d["xy"]))}
                       for d in drones],
            "footprints": [list(f) for f in self.footprints],
            "zones": [[list(p) for p in z] for z in self.zones],
            "tracks": [{"id": t.marker_id, "xy": [float(t.xy[0]), float(t.xy[1])],
                        "velocity": [float(t.velocity[0]), float(t.velocity[1])],
                        "behaviour": t.behaviour, "last_seen": float(t.last_seen)}
                       for t in self.taskboard.tracks()],
            "belief": self._heatmap(),
            "scoreboard": self.scoreboard(drones),
            "alarms": self.alarms(drones),
        }

    def to_json(self, drones: Sequence[dict], clock: float = 0.0) -> str:
        return json.dumps(self.snapshot(drones, clock))

    # -- evidence export -------------------------------------------------- #
    def export_evidence(self, out_dir, drones: Optional[Sequence[dict]] = None,
                        clock: float = 0.0) -> dict:
        """Write the evidence bundle: one annotated PNG per tagged id + a results sheet +
        a map snapshot. Returns the manifest."""
        import cv2
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        ev = self.state.evidence()
        images: List[str] = []
        for mid in sorted(ev):
            e = ev[mid]
            path = out / f"rover_{mid}.png"
            if e.frame is not None:
                bgr = cv2.cvtColor(np.asarray(e.frame), cv2.COLOR_RGB2BGR)
                cv2.putText(bgr, f"id {mid}", (8, 26), cv2.FONT_HERSHEY_SIMPLEX,
                            0.9, (0, 255, 0), 2)
            else:
                bgr = np.zeros((64, 64, 3), np.uint8)
            cv2.imwrite(str(path), bgr)
            images.append(str(path))
        sheet = out / "results.csv"
        lines = ["marker_id,north,east,time"]
        for mid in sorted(ev):
            e = ev[mid]
            xy = e.xy if e.xy is not None else (None, None)
            lines.append(f"{mid},{xy[0]},{xy[1]},{e.t}")
        sheet.write_text("\n".join(lines) + "\n")
        manifest = {"images": images, "results_sheet": str(sheet), "count": len(ev)}
        if drones is not None:
            map_path = out / "map.json"
            map_path.write_text(self.to_json(drones, clock))
            manifest["map"] = str(map_path)
        return manifest

    # -- operator overrides ---------------------------------------------- #
    def override(self, command: dict) -> dict:
        """Route an operator command into the Coordinator. Returns an ack."""
        if self.coordinator is None:
            return {"ok": False, "error": "no coordinator"}
        t = command.get("type")
        if t == "hold_all":
            self.coordinator.hold_all(True)
        elif t == "resume":
            self.coordinator.clear_all()
        elif t in ("retask", "designate"):
            role = Role[command["role"]] if t == "retask" else Role.TAG
            target = tuple(command["target"]) if command.get("target") is not None else None
            self.coordinator.force(int(command["tag_id"]), role, target)
        elif t == "recall":
            self.coordinator.force(int(command["tag_id"]), Role.HOLD, self.home_xy)
        else:
            return {"ok": False, "error": f"unknown command {t!r}"}
        return {"ok": True, "applied": t}
