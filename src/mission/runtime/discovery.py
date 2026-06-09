"""Drone discovery — abstract the sim's `pyhulax.discovery.Dola` vs the real standalone
`dola.py` (UDP 8668). Maps control `plane_id` ↔ UWB `tag_id`. In the sim the fixed config
IPs also work, so `fixed_ips` short-circuits discovery entirely (deterministic, no network).
"""

from __future__ import annotations

from typing import Dict, Optional


def _resolve_dola():
    try:
        from pyhulax.discovery import Dola          # sim (config-backed)
    except ImportError:
        from dola import Dola                        # real (standalone, UDP 8668)
    return Dola


class Discovery:
    def __init__(self, *, plane_to_tag: Optional[Dict[int, int]] = None,
                 fixed_ips: Optional[Dict[int, str]] = None,
                 listen_seconds: float = 2.0):
        self.plane_to_tag = dict(plane_to_tag) if plane_to_tag else {0: 0, 1: 1, 2: 2}
        self.fixed_ips = dict(fixed_ips) if fixed_ips else None
        self.listen_seconds = listen_seconds

    def get_all_ips(self) -> Dict[int, str]:
        """{plane_id: ip}. Uses fixed config IPs if given, else live Dola discovery."""
        if self.fixed_ips is not None:
            return dict(self.fixed_ips)
        Dola = _resolve_dola()
        d = Dola()
        if hasattr(d, "start"):
            d.start()
        try:
            return dict(d.get_all_ips(self.listen_seconds))
        finally:
            if hasattr(d, "stop"):
                d.stop()

    def tag_for_plane(self, plane_id: int) -> int:
        return self.plane_to_tag[plane_id]

    def resolve(self) -> Dict[int, str]:
        """{tag_id: ip} for the planes we know how to map to a UWB tag."""
        return {self.plane_to_tag[pid]: ip
                for pid, ip in self.get_all_ips().items()
                if pid in self.plane_to_tag}
