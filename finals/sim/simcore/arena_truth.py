"""Emit the authored arena as PLAIN GROUND-TRUTH DATA for the mission side.

Real teams are *given* the crate coordinates (the Discord file). To mirror
that, the sim emits its authored arena (dimensions + crates + archway) to a
plain YAML file the mission loads WITHOUT importing `simcore` — it is external
ground-truth data, exactly like the organisers' file, NOT a sim import. The
boundary holds: the mission reads a data file, not sim internals.

`build_arena_truth(cfg)` returns the plain dict; `write_arena_truth(cfg, path)`
writes the documented YAML. `scripts/emit_arena_truth.py` is the CLI wrapper,
and the committed `arena_truth.yaml` is generated from the default config.
"""

import yaml

_HEADER = (
    "# arena_truth.yaml — GROUND-TRUTH arena data for the MISSION side.\n"
    "#\n"
    "# Plain data, like the organisers' Discord coordinate file. The mission\n"
    "# loads THIS file (yaml.safe_load) — it must NOT import simcore.\n"
    "# Regenerate with:  python -m scripts.emit_arena_truth\n"
    "#\n"
    "# PROVISIONAL: mirrors sim_config.yaml arena.authored — a starting\n"
    "# estimate from the arena layout. Replace with the confirmed numbers.\n"
    "#\n"
    "# Frames: arena (north, east) m = competition (y, x). structures: center =\n"
    "# footprint centre, size = [north_m, east_m], height m (arch gates appear as\n"
    "# their two posts). landing_zones: coordinate-only (no marker), with valid.\n"
)


def build_arena_truth(cfg) -> dict:
    """The authored arena as a plain, simcore-free dict for the mission: arena
    dimensions, every structure's center/size/height (arch gates appear as
    their two posts), and the landing zones (coordinate-only, with valid)."""
    au = cfg.arena.authored
    return {
        "arena": {
            "length_m": float(cfg.arena.length_m),
            "width_m": float(cfg.arena.width_m),
            "height_m": float(cfg.arena.height_m),
        },
        "structures": [
            {
                "center": [float(cl.center[0]), float(cl.center[1])],
                "size": [float(cl.size[0]), float(cl.size[1])],
                "height": float(cl.height),
            }
            for cl in au.clusters
        ],
        "landing_zones": [
            {
                "id": int(pad.id),
                "north": float(pad.north),
                "east": float(pad.east),
                "valid": bool(pad.valid),
            }
            for pad in cfg.pads
        ],
    }


def write_arena_truth(cfg, path: str) -> dict:
    """Write the authored arena to `path` as documented YAML; returns the dict."""
    data = build_arena_truth(cfg)
    with open(path, "w") as f:
        f.write(_HEADER)
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
    return data
