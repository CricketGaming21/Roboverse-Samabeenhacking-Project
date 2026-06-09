"""Arena ground-truth loader — reads `config/arena_truth.yaml` (the sim emits it via
`emit_arena_truth`; on the day the Discord coordinate file plays the same role).

Loaded with **`yaml.safe_load` — NEVER imports `simcore`** (HARD invariant #10).
Frames: arena (north, east) metres; a crate `center` is its footprint centre, `size`
is the full footprint `[north_m, east_m]`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import yaml

_DEFAULT = (Path(__file__).resolve().parents[3] / "config" / "arena_truth.yaml")


@dataclass(frozen=True)
class Footprint:
    cn: float   # centre north (m)
    ce: float   # centre east (m)
    sn: float   # size north (m)
    se: float   # size east (m)
    height: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.cn, self.ce, self.sn, self.se)


@dataclass
class Arena:
    length_m: float            # North extent
    width_m: float             # East extent
    height_m: float = 3.0
    crates: List[Footprint] = field(default_factory=list)
    archway: Optional[dict] = None

    def footprint_tuples(self) -> List[Tuple[float, float, float, float]]:
        return [c.as_tuple() for c in self.crates]


def load_arena(path=None) -> Arena:
    p = Path(path) if path is not None else _DEFAULT
    data = yaml.safe_load(p.read_text())
    a = data["arena"]
    crates = []
    for c in data.get("crates", []) or []:
        cn, ce = c["center"]
        sn, se = c["size"]
        crates.append(Footprint(float(cn), float(ce), float(sn), float(se),
                                float(c.get("height", 0.0))))
    return Arena(length_m=float(a["length_m"]), width_m=float(a["width_m"]),
                 height_m=float(a.get("height_m", 3.0)), crates=crates,
                 archway=data.get("archway"))
