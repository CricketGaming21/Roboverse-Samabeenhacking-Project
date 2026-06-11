"""Evidence capture — the Phase-2 judge deliverable (R4).

On every bank we save an **annotated screenshot** (high-contrast detection box + an
`ID <n>` pill + a caption `drone <k> · t=<sim_s> · (<north>, <east>) m`) to
`logs/evidence/rover_<id>.png`, and (re)build a dark-theme `index.html` contact-sheet
gallery — the printable "inform the judge" artifact.

The frame is the **BGR** ndarray from `detector.frame_bgr` (real `.image` / sim `.to_rgb`),
so colours come out right on the real SDK. One file per id; the FIRST qualifying frame
wins (banking de-dups by id, so a good capture is never overwritten by a worse one).
"""

from __future__ import annotations

import html
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

Point = Tuple[float, float]

_BOX_BGR = (0, 230, 0)      # high-contrast green outline (BGR)
_PILL_BGR = (0, 230, 0)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def caption_for(drone_id: Optional[int], t: float, xy: Optional[Point]) -> str:
    """`drone <k> · t=<sim_s> · (<north>, <east>) m` — the canonical caption (HTML + overlay)."""
    if xy is not None and xy[0] is not None and xy[1] is not None:
        loc = f"({xy[0]:.2f}, {xy[1]:.2f}) m"
    else:
        loc = "(n/a)"
    dk = "?" if drone_id is None else int(drone_id)
    return f"drone {dk} · t={float(t):.1f}s · {loc}"


def annotate(frame_bgr: np.ndarray, bbox, marker_id: int, caption: str) -> np.ndarray:
    """Draw the detection box, an `ID <n>` pill and the caption strip on a COPY of the frame.

    Returns a new BGR ndarray (the input is never mutated)."""
    img = np.array(frame_bgr, copy=True)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    h, w = img.shape[:2]
    x, y, bw, bh = (int(round(float(v))) for v in bbox)

    # high-contrast detection box around the marker
    cv2.rectangle(img, (x, y), (x + bw, y + bh), _BOX_BGR, 3)

    # "ID <n>" pill, filled, just above the box (clamped into frame)
    label = f"ID {marker_id}"
    (tw, th), _ = cv2.getTextSize(label, _FONT, 0.7, 2)
    px = min(max(0, x), w - tw - 12)
    py = y - th - 12
    if py < 0:
        py = min(y + bh + 2, h - th - 12)
    cv2.rectangle(img, (px, py), (px + tw + 12, py + th + 10), _PILL_BGR, -1)
    cv2.putText(img, label, (px + 6, py + th + 4), _FONT, 0.7, (0, 0, 0), 2, cv2.LINE_AA)

    # caption strip across the bottom (black band + white text)
    band = 30
    cv2.rectangle(img, (0, h - band), (w, h), (0, 0, 0), -1)
    cv2.putText(img, caption, (8, h - 9), _FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


class EvidenceWriter:
    """Writes one annotated PNG per banked id + a dark-theme gallery to `out_dir`."""

    def __init__(self, out_dir="logs/evidence", *,
                 title: str = "RoboVerse C2 — Rover Evidence"):
        self.out_dir = Path(out_dir)
        self.title = title

    def _ensure(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)

    def write(self, annotated_bgr: np.ndarray, marker_id: int) -> str:
        """Persist an already-annotated BGR frame as `rover_<id>.png`. Returns the path."""
        self._ensure()
        path = self.out_dir / f"rover_{int(marker_id)}.png"
        cv2.imwrite(str(path), annotated_bgr)
        return str(path)

    def capture(self, frame_bgr: np.ndarray, bbox, marker_id: int,
                drone_id: Optional[int], t: float,
                xy: Optional[Point]) -> Tuple[np.ndarray, str]:
        """Annotate `frame_bgr` and write it. Returns `(annotated_bgr, path)`."""
        annotated = annotate(frame_bgr, bbox, marker_id, caption_for(drone_id, t, xy))
        return annotated, self.write(annotated, marker_id)

    def gallery(self, evidence: dict) -> str:
        """(Re)build the `index.html` contact sheet from a `{id: Evidence}` map. Returns its path."""
        self._ensure()
        cards = []
        for mid in sorted(evidence):
            e = evidence[mid]
            cap = html.escape(caption_for(getattr(e, "drone_id", None), e.t, e.xy))
            fname = Path(e.path).name if getattr(e, "path", None) else f"rover_{mid}.png"
            cards.append(
                f'    <figure class="card">\n'
                f'      <div class="thumb"><img src="{html.escape(fname)}" '
                f'alt="rover {mid}" loading="lazy"></div>\n'
                f'      <figcaption><span class="id">ID {int(mid)}</span>'
                f'<span class="cap">{cap}</span></figcaption>\n'
                f'    </figure>')
        count = len(evidence)
        ids = ", ".join(str(m) for m in sorted(evidence)) or "—"
        page = _GALLERY_TEMPLATE.format(
            title=html.escape(self.title), count=count, ids=html.escape(ids),
            cards="\n".join(cards) if cards else
            '    <p class="empty">No rovers banked yet.</p>')
        idx = self.out_dir / "index.html"
        idx.write_text(page)
        return str(idx)


_GALLERY_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<style>
  :root {{ color-scheme: dark; }}
  body {{ margin: 0; background: #0d1117; color: #e6edf3;
          font: 15px/1.5 -apple-system, Segoe UI, Roboto, sans-serif; }}
  header {{ padding: 20px 24px; border-bottom: 1px solid #21262d; }}
  header h1 {{ margin: 0 0 4px; font-size: 20px; }}
  header .sub {{ color: #7d8590; font-size: 13px; }}
  .grid {{ display: grid; gap: 16px; padding: 24px;
           grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); }}
  .card {{ margin: 0; background: #161b22; border: 1px solid #30363d;
           border-radius: 10px; overflow: hidden; }}
  .thumb {{ background: #010409; aspect-ratio: 4 / 3; display: flex; }}
  .thumb img {{ width: 100%; height: 100%; object-fit: contain; }}
  figcaption {{ padding: 10px 12px; display: flex; flex-direction: column; gap: 2px; }}
  .id {{ font-weight: 700; color: #3fb950; letter-spacing: .3px; }}
  .cap {{ color: #9da7b3; font-size: 13px; }}
  .empty {{ padding: 24px; color: #7d8590; }}
</style>
</head>
<body>
  <header>
    <h1>{title}</h1>
    <div class="sub">{count} distinct rover id(s) banked &middot; ids: {ids}</div>
  </header>
  <main class="grid">
{cards}
  </main>
</body>
</html>
"""
