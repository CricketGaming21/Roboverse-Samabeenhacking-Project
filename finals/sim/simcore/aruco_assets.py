"""Generate ArUco marker PNGs (pads + rovers) with a white quiet-zone border.

Simulator INTERNAL — mission code must never import simcore.

Markers are generated from config.aruco.dictionary (default DICT_6X6_250 —
matches the organiser sample) at high resolution with a white quiet zone of
one module per side, so cv2.aruco decodes them both from the PNG and from
rendered camera frames. PNGs land in assets/ (gitignored, regenerated on
demand at world boot or via scripts/gen_assets.py).
"""

from pathlib import Path

import cv2

# 6x6 dictionary marker = 6 data + 2 black-border modules = 8 modules/side.
# Quiet zone = 1 module per side => MARKER_PX/8 of white border.
MARKER_PX = 512
QUIET_DIVISOR = 8


def get_dictionary(name: str):
    """cv2.aruco predefined dictionary from its config name."""
    if not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary: {name!r}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def texture_scale() -> float:
    """Total texture side / marker side. A pad textured with one of these
    PNGs must be marker_size_m * texture_scale() wide for the printed marker
    (black border included) to measure exactly marker_size_m."""
    quiet = MARKER_PX // QUIET_DIVISOR
    return (MARKER_PX + 2 * quiet) / MARKER_PX


def marker_image(dict_name: str, marker_id: int, marker_px: int = MARKER_PX):
    """BGR image: the marker surrounded by its white quiet zone."""
    dictionary = get_dictionary(dict_name)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_px)
    quiet = marker_px // QUIET_DIVISOR
    bordered = cv2.copyMakeBorder(marker, quiet, quiet, quiet, quiet,
                                  cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(bordered, cv2.COLOR_GRAY2BGR)


def marker_path(dict_name: str, marker_id: int,
                out_dir: str = "assets") -> Path:
    return Path(out_dir) / f"aruco_{dict_name}_id{marker_id}.png"


def ensure_marker_png(cfg, marker_id: int, out_dir: str = "assets",
                      force: bool = False) -> str:
    """Write the marker PNG if missing (or force) and return its path."""
    path = marker_path(cfg.aruco.dictionary, marker_id, out_dir)
    if force or not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), marker_image(cfg.aruco.dictionary,
                                                   marker_id)):
            raise RuntimeError(f"failed to write marker PNG: {path}")
    return str(path)


# Unit quad (XY plane, +Z normal) with explicit 0..1 UVs. PyBullet's GEOM_BOX
# auto-UVs crop/zoom textures, which breaks marker decoding — textured marker
# faces must use this mesh so the PNG maps exactly once across the face.
_QUAD_OBJ = """\
v -0.5 -0.5 0
v 0.5 -0.5 0
v 0.5 0.5 0
v -0.5 0.5 0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""


def ensure_quad_obj(out_dir: str = "assets") -> str:
    """Write (once) and return the unit textured-quad mesh path."""
    path = Path(out_dir) / "unit_quad.obj"
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(_QUAD_OBJ)
    return str(path)


def generate_all(cfg, out_dir: str = "assets", force: bool = True) -> dict:
    """(Re)generate every configured marker (pad ids + rover ids).

    Returns {marker_id: png_path}.
    """
    ids = [pad.id for pad in cfg.pads] + list(cfg.rovers.marker_ids)
    if len(set(ids)) != len(ids):
        raise ValueError(f"pad/rover marker ids must be distinct: {ids}")
    return {mid: ensure_marker_png(cfg, mid, out_dir, force=force)
            for mid in ids}
