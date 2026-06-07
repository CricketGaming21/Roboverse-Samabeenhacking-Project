"""DebugProbe (simcore/debug.py) — read-only introspection sanity (headless).

The probe is a sim-internal tool for tests/debug scripts; this is also its
usage example. It must report consistent multi-frame state and never mutate
anything.
"""

import json
import time

import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from simcore.config import load_config
from simcore.debug import DebugProbe, start_debug_loop
from simcore.registry import get_registry, shutdown_registry


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.arena.obstacles.count = 0
    c.rovers.patrol.speed_mps = 0.0  # parked: stable read-only comparisons
    return c


@pytest.fixture()
def sim(cfg):
    reg = get_registry(cfg)
    yield reg
    shutdown_registry()


def test_probe_snapshot_contents_and_json(sim, cfg):
    d = DroneAPI()
    d.connect(cfg.drones.units[0].ip)
    d.takeoff(150)
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.move_to(0, 240 - cfg.camera.mount_offset_m * 100, 150)  # over pad 10
    time.sleep(0.3)  # let the referee accumulate holds

    probe = DebugProbe(sim)
    snap = probe.snapshot()

    # drone: true + estimate in all frames at once, sensors, goal state
    d0 = snap["drones"][0]
    assert d0["flying"] is True and d0["connected"] is True
    assert abs(d0["true"]["world"][2] - 1.5) < 0.03
    assert abs(d0["true"]["arena_ne_m"][0] - 3.0) < 0.15   # over pad 10
    assert d0["true"]["takeoff_cm"] is not None
    assert d0["estimate"]["takeoff_cm"] is not None
    assert d0["estimate"]["drift_error_m"] >= 0.0
    assert d0["goal"] is None and d0["executing"] is False  # hovering idle
    rays = d0["sensors"]["rays"]
    assert set(rays) == {"forward", "back", "left", "right", "down"}
    for r in rays.values():
        assert len(r["start"]) == 3 and len(r["end"]) == 3
    assert abs(d0["sensors"]["altitude_cm"] - 150) < 3

    # rovers: id + position (+ waypoint None while parked)
    assert [r["marker_id"] for r in snap["rovers"]] == \
        list(cfg.rovers.marker_ids)[:cfg.rovers.count]
    assert all(r["waypoint_arena_ne_m"] is None for r in snap["rovers"])

    # referee: WHY pad 10 is scoring — size, frame, hold, banked
    info = snap["referee"]["per_drone"][0].get(10)
    assert info is not None, "probe referee view should see pad 10"
    assert info["side_px"] >= cfg.scoring.min_marker_px
    assert info["fully_in_frame"] and info["gate_ok"]
    assert info["hold"] >= 1
    assert info["hold_needed"] == cfg.scoring.hold_frames

    # monitor: commands + recent timestamps
    mon = snap["monitor"]
    assert mon["commands"].get(0) == 2  # takeoff + move_to
    assert len(mon["recent_commands"]["0"]
               if "0" in mon["recent_commands"]
               else mon["recent_commands"][0]) == 2

    # JSON round-trip for logging/diffing
    parsed = json.loads(probe.to_json(snap))
    assert parsed["sim_time"] == snap["sim_time"]
    assert probe.format_text(snap).startswith("[t=")


def test_probe_is_read_only(sim, cfg):
    d = DroneAPI()
    d.connect(cfg.drones.units[0].ip)
    d.takeoff(100)
    probe = DebugProbe(sim)
    pose_before, yaw_before = sim.drone_world_pose(0)
    for _ in range(3):
        probe.snapshot()
    pose_after, yaw_after = sim.drone_world_pose(0)
    assert pose_before == pose_after and yaw_before == yaw_after
    assert sim.run_on_sim_thread(lambda: sim.drones[0].goal) is None


def test_debug_loop_dumps_jsonl(sim, cfg, tmp_path):
    d = DroneAPI()
    d.connect(cfg.drones.units[0].ip)
    d.takeoff(100)
    out = tmp_path / "dump.jsonl"
    stop = start_debug_loop(sim, print_text=False, dump_path=str(out),
                            period_sim_s=1.0)
    time.sleep(0.5)  # ~5 sim seconds at rtf 10
    stop()
    lines = out.read_text().strip().splitlines()
    assert len(lines) >= 2
    for line in lines:
        snap = json.loads(line)
        assert "sim_time" in snap and "drones" in snap and "monitor" in snap
