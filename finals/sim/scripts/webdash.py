"""Graphical WEB dashboard — the SSH-friendly live monitor (--dashboard).

OBSERVER-ONLY and FREEZE-PROOF BY CONSTRUCTION. This is a small headless HTTP
server: it reads `simcore.DebugProbe.snapshot()` (never pyhulax) and renders
the arena + per-drone camera frames through the SAME offscreen EGL path the
recorder uses (`registry.render_arena` / `render_camera`) — there is NO
`p.GUI` anywhere, so it cannot hit the getCameraImage-vs-GUI WSLg deadlock. It
reaches PC-B because it is a network service, not a window.

It MUTATES NOTHING: every endpoint only reads probe / registry state (the
camera renders go through the existing passive path, which changes no world
state), so probe-invariance / scoreboard-equality still holds.

Endpoints:
  GET /                  HTML/JS page that polls /api/state (~poll_hz) and
                         renders, per drone, a graphical panel: telemetry
                         (speed, UWB pos + drifting estimate, heading, camera
                         pitch, yaw/pitch/roll, command-or-sticks, battery), a
                         UWB-OK badge, and a car-style 5-segment proximity
                         widget; plus the scoreboard / phase banner.
  GET /api/state         JSON snapshot from DebugProbe (read-only).
  GET /api/arena.jpg     latest offscreen third-person arena frame (EGL).
  GET /api/camera/<i>.jpg latest offscreen frame from drone i's camera, with
                         the real cv2.aruco detect/acquire overlay.

Access from PC-B (see README): browse http://<tailscale-ip>:<port> directly,
or SSH-forward `ssh -L <port>:localhost:<port> drone@<host> -p 2222` then open
http://localhost:<port>.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from simcore import camfeed
from simcore.debug import DebugProbe
from simcore.log import get_logger


def _make_handler(dashboard):
    class _Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):    # silence default stderr spam
            pass

        def do_GET(self):
            dashboard.handle_get(self)

    return _Handler


class WebDashboard:
    """Headless HTTP server serving the live graphical dashboard. Read-only.

    Renders are TTL-cached (dashboard.min_render_period_s) so polling and
    image refresh cannot hammer the sim thread regardless of client count."""

    def __init__(self, registry, host=None, port=None):
        self._reg = registry
        self._cfg = registry.config
        self._dc = self._cfg.dashboard
        self._probe = DebugProbe(registry)
        self._log = get_logger("webdash", self._cfg)
        self._host = self._dc.host if host is None else host
        self._port = int(self._dc.port if port is None else port)
        self._server = None
        self._thread = None
        self._lock = threading.Lock()
        self._cache = {}            # key -> (monotonic_time, body_bytes, ctype)

    @property
    def port(self) -> int:
        return self._port

    @property
    def url(self) -> str:
        return f"http://{self._host}:{self._port}/"

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> "WebDashboard":
        ThreadingHTTPServer.allow_reuse_address = True
        self._server = ThreadingHTTPServer((self._host, self._port),
                                           _make_handler(self))
        self._server.daemon_threads = True
        self._port = self._server.server_address[1]   # actual port (if 0)
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        name="hula-webdash", daemon=True)
        self._thread.start()
        self._log.info("web dashboard on http://%s:%d  (browse it from PC-B; "
                       "see README for the SSH tunnel)", self._host,
                       self._port)
        return self

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None

    # ------------------------------------------------------------------ #
    # Request dispatch
    # ------------------------------------------------------------------ #

    def handle_get(self, h) -> None:
        path = h.path.split("?", 1)[0]
        try:
            if path in ("/", "/index.html"):
                self._respond(h, 200, "text/html; charset=utf-8",
                              self.page_html().encode("utf-8"))
            elif path == "/api/state":
                self._respond(h, 200, "application/json",
                              self.state_json().encode("utf-8"))
            elif path == "/api/arena.jpg":
                self._image(h, self.arena_jpg())
            elif path.startswith("/api/camera/") and path.endswith(".jpg"):
                try:
                    idx = int(path[len("/api/camera/"):-len(".jpg")])
                except ValueError:
                    self._respond(h, 404, "text/plain", b"bad camera index")
                    return
                if not (0 <= idx < len(self._reg.drones)):
                    self._respond(h, 404, "text/plain", b"no such drone")
                    return
                self._image(h, self.camera_jpg(idx))
            else:
                self._respond(h, 404, "text/plain", b"not found")
        except (RuntimeError, TimeoutError):    # sim shutting down
            self._respond(h, 503, "text/plain", b"sim unavailable")
        except Exception as exc:                # never kill the server thread
            self._respond(h, 500, "text/plain", str(exc).encode("utf-8"))

    def _respond(self, h, code, ctype, body) -> None:
        h.send_response(code)
        h.send_header("Content-Type", ctype)
        h.send_header("Content-Length", str(len(body)))
        h.send_header("Cache-Control", "no-store")
        h.end_headers()
        if body:
            h.wfile.write(body)

    def _image(self, h, body) -> None:
        if body is None:
            self._respond(h, 503, "text/plain", b"cameras disabled")
        else:
            self._respond(h, 200, "image/jpeg", body)

    # ------------------------------------------------------------------ #
    # State (read-only; telemetry/proximity == DebugProbe.snapshot())
    # ------------------------------------------------------------------ #

    def state_dict(self) -> dict:
        """A DebugProbe snapshot (telemetry + sensors/proximity), augmented
        with the scoreboard/phase banner. referee_view is skipped (its
        per-marker scan render is not shown here and would re-render every
        poll); the cheap score/banked come straight from the referee."""
        snap = self._probe.snapshot(referee_view=False)
        reg = self._reg
        snap["phase"] = reg.scenario.phase if reg.scenario else None
        snap["landing_score"] = reg.landing_scorer.score()
        ref = reg.referee
        if ref is not None:
            snap["referee"].update(
                score=ref.score(),
                banked=sorted(int(m) for m in ref.banked_ids()))
        snap["serve_images"] = bool(self._dc.serve_images
                                    and reg.cameras_enabled)
        return snap

    def state_json(self) -> str:
        return self._probe.to_json(self.state_dict())

    # ------------------------------------------------------------------ #
    # Offscreen images (EGL, sim thread — NEVER p.GUI), TTL-cached
    # ------------------------------------------------------------------ #

    def _cached(self, key, producer):
        period = max(0.0, float(self._dc.min_render_period_s))
        now = time.monotonic()
        with self._lock:
            hit = self._cache.get(key)
            if hit is not None and now - hit[0] < period:
                return hit[1]
        body = producer()
        with self._lock:
            self._cache[key] = (time.monotonic(), body)
        return body

    def _encode_jpg(self, bgr):
        ok, buf = cv2.imencode(
            ".jpg", bgr,
            [cv2.IMWRITE_JPEG_QUALITY, int(self._dc.jpeg_quality)])
        return buf.tobytes() if ok else None

    def arena_jpg(self):
        if not (self._dc.serve_images and self._reg.cameras_enabled):
            return None

        def _produce():
            rgb = self._reg.render_arena()
            if rgb is None:
                return None
            return self._encode_jpg(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        return self._cached("arena", _produce)

    def camera_jpg(self, idx):
        if not (self._dc.serve_images and self._reg.cameras_enabled):
            return None

        def _produce():
            drone = self._reg.drones[idx]
            if not getattr(drone, "flying", False):
                return None
            rgb = self._reg.render_camera(drone)
            if rgb is None:
                return None
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ref = self._reg.referee
            banked = sorted(ref.banked_ids()) if ref is not None else []
            overlaid, _found = camfeed.acquisition_overlay(
                self._cfg, bgr, banked_ids=banked)
            return self._encode_jpg(overlaid)
        return self._cached(f"cam{idx}", _produce)

    # ------------------------------------------------------------------ #
    # The page (HTML/JS) — polls /api/state and refreshes the images
    # ------------------------------------------------------------------ #

    def page_html(self) -> str:
        return (_PAGE
                .replace("__POLL_HZ__", repr(float(self._dc.poll_hz)))
                .replace("__IMG_HZ__", repr(float(self._dc.image_refresh_hz))))


# --------------------------------------------------------------------------- #
# Static page. Telemetry + proximity come from /api/state; the arena + camera
# images are <img> tags refreshed with a cache-busting query param.
# --------------------------------------------------------------------------- #

_PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HULA sim — live dashboard</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif;
         background:#0d1117; color:#e6edf3; }
  header { padding:10px 16px; background:#161b22; border-bottom:1px solid #30363d;
           display:flex; gap:18px; align-items:center; flex-wrap:wrap; }
  header b { font-size:16px; }
  .pill { padding:2px 10px; border-radius:12px; background:#21262d; }
  .pill.phase { background:#1f6feb; }
  #err { color:#f85149; margin-left:auto; }
  .wrap { padding:14px; display:grid; gap:14px;
          grid-template-columns: 1fr; }
  @media (min-width:1100px){ .wrap { grid-template-columns: 420px 1fr; } }
  .arena img, .feed { width:100%; border-radius:8px; background:#000;
                      display:block; border:1px solid #30363d; }
  .arena h3,.drones h3 { margin:0 0 6px; font-size:13px; color:#8b949e;
                         text-transform:uppercase; letter-spacing:.06em; }
  .grid { display:grid; gap:12px;
          grid-template-columns: repeat(auto-fill,minmax(320px,1fr)); }
  .card { background:#161b22; border:1px solid #30363d; border-radius:10px;
          overflow:hidden; }
  .card .top { display:flex; align-items:center; gap:8px; padding:8px 10px;
               background:#10151c; border-bottom:1px solid #30363d; }
  .card .top .title { font-weight:600; }
  .badge { padding:1px 8px; border-radius:10px; font-size:12px; }
  .badge.ok { background:#1a7f37; } .badge.bad { background:#9e2f2f; }
  .badge.state { background:#30363d; }
  .body { display:grid; grid-template-columns: 1fr 130px; gap:10px; padding:10px; }
  table.tel { width:100%; border-collapse:collapse; }
  table.tel td { padding:1px 0; vertical-align:top; }
  table.tel td.k { color:#8b949e; padding-right:8px; white-space:nowrap; }
  table.tel td.v { font-variant-numeric:tabular-nums; }
  .feedwrap { padding:0 10px 10px; }
  svg.prox { width:130px; height:130px; }
  .seg { fill:#23402b; stroke:#0d1117; stroke-width:2; }
  .seg.on { fill:#f85149; }
  .hub { fill:#30363d; stroke:#0d1117; stroke-width:2; }
  .proxlbl { fill:#8b949e; font-size:11px; }
  .note { color:#8b949e; padding:10px; }
</style></head>
<body>
<header>
  <b>HULA sim</b>
  <span class="pill phase" id="phase">—</span>
  <span class="pill">t <span id="t">0.0</span>s</span>
  <span class="pill">landings <span id="land">0</span>/3</span>
  <span class="pill">rovers scanned <span id="scan">0</span>/5</span>
  <span class="pill">ids <span id="ids">-</span></span>
  <span id="err"></span>
</header>
<div class="wrap">
  <div class="arena" id="arena-col">
    <h3>Arena (offscreen 3D)</h3>
    <img class="feed" id="arena" alt="arena">
  </div>
  <div class="drones">
    <h3>Drones</h3>
    <div class="grid" id="grid"></div>
  </div>
</div>
<script>
const POLL_MS = Math.max(150, Math.round(1000/__POLL_HZ__));
const IMG_MS  = Math.max(250, Math.round(1000/__IMG_HZ__));
let SERVE_IMAGES = false, built = false;
const DIRS = [["forward","F",55,8],["back","B",55,102],
              ["left","L",8,55],["right","R",102,55]];

function proxSvg(i){
  let r = '';
  for (const [d,l,x,y] of DIRS){
    r += `<rect id="d${i}-p-${d}" class="seg" x="${x}" y="${y}" width="55" height="20" rx="4"></rect>`;
    r += `<text class="proxlbl" x="${x+27}" y="${y+15}" text-anchor="middle">${l}</text>`;
  }
  r += `<rect id="d${i}-p-down" class="hub" x="50" y="50" width="30" height="30" rx="5"></rect>`;
  r += `<text class="proxlbl" x="65" y="69" text-anchor="middle">D</text>`;
  return `<svg class="prox" viewBox="0 0 130 130">${r}</svg>`;
}
const ROWS = [["spd","speed (m/s)"],["uwb","UWB n,e (m)"],["est","est n,e (cm)"],
              ["drift","drift (cm)"],["hdg","heading (deg)"],["camp","cam pitch"],
              ["ypr","yaw/pitch/roll"],["cmd","command"],["batt","battery"]];
function card(d){
  const i = d.index;
  let tel = '';
  for (const [k,lbl] of ROWS)
    tel += `<tr><td class="k">${lbl}</td><td class="v" id="d${i}-${k}">—</td></tr>`;
  const feed = SERVE_IMAGES
    ? `<div class="feedwrap"><img class="feed" id="d${i}-camimg" data-base="/api/camera/${i}.jpg" alt="cam ${i}"></div>` : '';
  return `<div class="card"><div class="top">
      <span class="title">drone ${i}</span>
      <span class="badge state" id="d${i}-state">—</span>
      <span class="badge" id="d${i}-uwb">UWB</span>
      <span class="badge" id="d${i}-viol" style="display:none">VIOLATION</span>
      <span style="margin-left:auto;color:#8b949e" id="d${i}-ip"></span>
    </div><div class="body"><table class="tel"><tbody>${tel}</tbody></table>
    ${proxSvg(i)}</div>${feed}</div>`;
}
const set=(id,v)=>{const e=document.getElementById(id); if(e) e.textContent=v;};
const f=(x,n=2)=>(x==null?'—':Number(x).toFixed(n));
function build(s){
  SERVE_IMAGES = !!s.serve_images;
  document.getElementById('grid').innerHTML = s.drones.map(card).join('');
  if(!SERVE_IMAGES) document.getElementById('arena-col').style.display='none';
  built = true;
}
function update(s){
  set('phase', (s.phase||'—').toUpperCase());
  set('t', f(s.sim_time,1));
  set('land', s.landing_score ?? 0);
  const ref = s.referee||{};
  set('scan', ref.score ?? 0);
  set('ids', (ref.banked&&ref.banked.length)? ref.banked.join(',') : '-');
  for (const d of s.drones){
    const i=d.index, ne=d.true.arena_ne_m, est=d.estimate.takeoff_cm,
          o=d.orientation_deg;
    set(`d${i}-ip`, d.ip);
    const st = d.flying?'FLY':(d.connected?'RDY':'OFF');
    set(`d${i}-state`, `${st} ${d.mode}`);
    set(`d${i}-spd`, f(d.speed_mps));
    set(`d${i}-uwb`, `${f(ne[0])}, ${f(ne[1])}`);
    set(`d${i}-est`, est?`${f(est[0],0)}, ${f(est[1],0)}`:'—');
    set(`d${i}-drift`, f(d.estimate.drift_error_m*100,1));
    set(`d${i}-hdg`, f(o.yaw,1));
    set(`d${i}-camp`, f(d.camera_pitch_deg,0));
    set(`d${i}-ypr`, `${f(o.yaw,0)}/${f(o.pitch,0)}/${f(o.roll,0)}`);
    set(`d${i}-cmd`, cmdText(d));
    set(`d${i}-batt`, f(d.battery_pct,0)+'%');
    // UWB-OK badge
    const ub=document.getElementById(`d${i}-uwb`);
    if(ub){ ub.textContent = d.uwb_ok?'UWB OK':'UWB --';
            ub.className='badge '+(d.uwb_ok?'ok':'bad'); }
    // compliance VIOLATION badge (no-fly-over-crate / altitude cap)
    const c=d.compliance||{}, vb=document.getElementById(`d${i}-viol`);
    if(vb){ if(c.violation){ vb.style.display='';
              vb.className='badge bad';
              vb.textContent='VIOLATION: '+(c.over_obstacle?'OVER CRATE':'ALT CAP');
            } else { vb.style.display='none'; } }
    // proximity segments (boolean barrier flags only)
    const rays=d.sensors.rays;
    for(const k of ['forward','back','left','right','down']){
      const seg=document.getElementById(`d${i}-p-${k}`);
      if(seg) seg.classList.toggle('on', !!(rays[k]&&rays[k].blocked));
    }
  }
}
function cmdText(d){
  const g=d.goal;
  if(!g) return 'idle';
  if(d.mode==='manual'&&g.stick){const s=g.stick;
    return `sticks F${s.forward.toFixed(2)} R${s.right.toFixed(2)} U${s.up.toFixed(2)} Y${s.rotate.toFixed(2)}`;}
  const t=g.target_world;
  let s=g.kind; if(t) s+=` → ${t[0].toFixed(1)},${t[1].toFixed(1)},${t[2].toFixed(1)}`;
  if(g.remaining_m!=null) s+=` (${g.remaining_m.toFixed(2)} m)`;
  return s;
}
async function poll(){
  try{
    const r=await fetch('/api/state',{cache:'no-store'});
    const s=await r.json();
    if(!built) build(s);
    update(s);
    document.getElementById('err').textContent='';
  }catch(e){ document.getElementById('err').textContent='disconnected'; }
}
setInterval(poll, POLL_MS); poll();
setInterval(()=>{
  if(!SERVE_IMAGES) return;
  const t=Date.now();
  const a=document.getElementById('arena'); if(a) a.src='/api/arena.jpg?t='+t;
  document.querySelectorAll('img.feed[data-base]').forEach(img=>{
    img.src=img.dataset.base+'?t='+t; });
}, IMG_MS);
</script></body></html>
"""
