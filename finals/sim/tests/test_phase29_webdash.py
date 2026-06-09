"""Phase 29 acceptance test — `--gui` 3D-only safe view + graphical WEB
dashboard (headless; no window, no browser).

Boundaries proven here: `--gui` runs the camera-disabled freeze-proof mode
(no getCameraImage path active); the web dashboard serves the page + a
read-only DebugProbe JSON (telemetry/proximity matching the probe) + valid
offscreen JPEGs; it perturbs nothing (probe-invariance); it never connects
p.GUI; and nothing is added to the public pyhulax/UWB surface.
"""

import inspect
import json
import urllib.error
import urllib.request

import pytest

from pyhulax import DroneAPI

from scripts.webdash import WebDashboard
from simcore.config import load_config
from simcore.debug import DebugProbe
from simcore.registry import get_registry, shutdown_registry


def _cfg(rovers=2):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = rovers
    c.motion.realistic = False
    return c


def _get(port, path, timeout=10):
    """GET -> (status, content_type, body bytes). HTTP errors return their
    code (so 404/503 are testable, not exceptions)."""
    url = f"http://127.0.0.1:{port}{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.headers.get("Content-Type", ""), r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get("Content-Type", ""), e.read()


# --------------------------------------------------------------------------- #
# A. --gui is the freeze-proof 3D-only view (cameras OFF)
# --------------------------------------------------------------------------- #

def test_gui_wires_camera_disabled_freeze_proof_mode():
    # both launchers must build the registry with cameras OFF under --gui
    # (the Phase-27/21b freeze-proof combination), so no getCameraImage path
    # is reachable while a p.GUI window is open.
    import scripts.run_sim as run_sim
    import scripts.scenario_demo as demo
    assert "cameras_enabled=not args.gui" in inspect.getsource(run_sim.main)
    assert "cameras_enabled=not gui" in inspect.getsource(
        demo.run_scenario_demo)


def test_camera_disabled_registry_renders_nothing():
    # functional proof of the mode --gui selects: every camera-render entry
    # point refuses and the scanning referee never starts.
    cfg = _cfg()
    cfg.scoring.enabled = True
    reg = get_registry(cfg, cameras_enabled=False)
    try:
        assert reg.cameras_enabled is False
        assert reg.referee is None              # no scanning thread
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        assert reg.render_camera(reg.drones[0]) is None
        assert reg.render_arena(160, 120) is None
        # the dashboard's /api/state still works in this mode (no render)
        wd = WebDashboard(reg, host="127.0.0.1", port=0)
        st = wd.state_dict()
        assert len(st["drones"]) == len(cfg.drones.units)
        assert st["serve_images"] is False      # images off when cameras off
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# B. The web dashboard: page + state + images
# --------------------------------------------------------------------------- #

def test_dashboard_serves_page_state_and_images():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        wd = WebDashboard(reg, host="127.0.0.1", port=0).start()
        try:
            # GET / -> the HTML/JS page
            s, ct, body = _get(wd.port, "/")
            assert s == 200 and "text/html" in ct
            page = body.decode("utf-8")
            assert "HULA" in page and "/api/state" in page
            assert "prox" in page                  # proximity widget present

            # GET /api/state -> JSON whose per-drone telemetry + proximity
            # fields match a fresh DebugProbe.snapshot()
            s, ct, body = _get(wd.port, "/api/state")
            assert s == 200 and "application/json" in ct
            state = json.loads(body)
            probe_snap = DebugProbe(reg).snapshot(referee_view=False)
            assert len(state["drones"]) == len(probe_snap["drones"])
            for sd, pd in zip(state["drones"], probe_snap["drones"]):
                # identical schema (state is literally a probe snapshot)
                assert set(sd.keys()) == set(pd.keys())
                # telemetry fields the panel shows are all present
                for k in ("speed_mps", "estimate", "orientation_deg",
                          "camera_pitch_deg", "uwb_ok", "battery_pct",
                          "true", "goal", "mode", "sensors"):
                    assert k in sd
                # proximity = the five boolean barrier flags, matching probe
                rays = sd["sensors"]["rays"]
                assert set(rays) == {"forward", "back", "left", "right", "down"}
                for dirn in rays:
                    assert isinstance(rays[dirn]["blocked"], bool)
                    assert rays[dirn]["blocked"] == \
                        pd["sensors"]["rays"][dirn]["blocked"]
            # banner fields
            assert "phase" in state and "landing_score" in state
            assert state["serve_images"] is True

            # GET /api/arena.jpg and /api/camera/0.jpg -> valid offscreen JPEGs
            s, ct, body = _get(wd.port, "/api/arena.jpg")
            assert s == 200 and ct == "image/jpeg"
            assert body[:3] == b"\xff\xd8\xff"          # JPEG SOI
            s, ct, body = _get(wd.port, "/api/camera/0.jpg")
            assert s == 200 and ct == "image/jpeg"
            assert body[:3] == b"\xff\xd8\xff"

            # a non-existent drone index is a clean 404 (not a crash)
            s, _, _ = _get(wd.port, "/api/camera/99.jpg")
            assert s == 404
        finally:
            wd.stop()
    finally:
        shutdown_registry()


def test_dashboard_is_read_only_does_not_perturb_the_sim():
    cfg = _cfg()
    cfg.scoring.enabled = False
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        d.hover(0.3)                       # crisp + no goal => pose is frozen
        pose0, yaw0 = reg.drone_world_pose(0)
        wd = WebDashboard(reg, host="127.0.0.1", port=0).start()
        try:
            for _ in range(8):             # hammer every endpoint
                _get(wd.port, "/api/state")
                _get(wd.port, "/api/arena.jpg")
                _get(wd.port, "/api/camera/0.jpg")
        finally:
            wd.stop()
        pose1, yaw1 = reg.drone_world_pose(0)
        # serving the dashboard mutated NOTHING: idle pose + goal untouched
        assert pose0 == pose1 and yaw0 == yaw1
        assert reg.run_on_sim_thread(lambda: reg.drones[0].goal) is None
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# C. Boundaries — no p.GUI, no pyhulax, no new public surface
# --------------------------------------------------------------------------- #

def test_dashboard_never_connects_gui_and_uses_offscreen_only():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        assert reg.gui is False                # headless DIRECT, never p.GUI
        wd = WebDashboard(reg, host="127.0.0.1", port=0).start()
        try:
            assert _get(wd.port, "/api/arena.jpg")[0] == 200  # served offscreen
        finally:
            wd.stop()
        # the dashboard never touches pybullet at all: it can only render
        # through the registry's offscreen path (which is gated by
        # cameras_enabled), so it cannot connect a p.GUI window itself.
        import scripts.webdash as mod
        imports = [ln for ln in inspect.getsource(mod).splitlines()
                   if ln.lstrip().startswith(("import ", "from "))]
        assert not any("pybullet" in ln for ln in imports)
    finally:
        shutdown_registry()


def test_dashboard_reads_simcore_only_never_pyhulax():
    import scripts.webdash as mod
    imports = [ln for ln in inspect.getsource(mod).splitlines()
               if ln.lstrip().startswith(("import ", "from "))]
    assert any("simcore" in ln for ln in imports)
    assert not any("pyhulax" in ln for ln in imports)


def test_no_new_public_surface():
    import pyhulax
    import pyhulax.core
    from UWBParserThread import UWBParserThread
    public = (set(dir(pyhulax)) | set(dir(pyhulax.DroneAPI))
              | set(dir(pyhulax.core)) | set(dir(UWBParserThread)))
    leaked = [n for n in public if "dashboard" in n.lower()
              or "webdash" in n.lower() or "http" in n.lower()]
    assert leaked == []
