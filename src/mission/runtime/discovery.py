"""Drone discovery — resolve control IP ↔ UWB tag id.

**Config-first.** In the sim, `pyhulax.discovery.Dola` imports but every method raises
`NotImplementedError`, so we DO NOT call it: the IP↔tag map comes straight from
`config.drones` (which mirrors the sim's `drones.units`). On the real day, pass
`use_dola=True` to broadcast-discover live IPs via the standalone `dola.py` (UDP 8668);
if that `Dola` is the sim stub (raises `NotImplementedError`), we transparently fall back
to the config map. `plane_id` (discovery key) ↔ `tag_id` (UWB) is configurable (identity
by default).
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple


def _resolve_dola():
    try:
        from pyhulax.discovery import Dola          # sim ships this (methods raise)
    except ImportError:
        from dola import Dola                        # real day: standalone UDP listener
    return Dola


class Discovery:
    def __init__(self, units: Sequence, *, use_dola: bool = False,
                 plane_to_tag: Optional[Dict[int, int]] = None,
                 listen_seconds: float = 5.0):
        """`units` = config drone units (each with `.ip` and `.tag_id`). `use_dola`
        opts into live broadcast discovery for the real day."""
        self._units: List[Tuple[str, int]] = [(u.ip, u.tag_id) for u in units]
        self.use_dola = bool(use_dola)
        self.plane_to_tag = dict(plane_to_tag) if plane_to_tag else \
            {tag: tag for _ip, tag in self._units}     # plane_id == tag_id by default
        self.listen_seconds = listen_seconds

    @classmethod
    def from_config(cls, cfg, *, use_dola: bool = False,
                    plane_to_tag: Optional[Dict[int, int]] = None) -> "Discovery":
        return cls(cfg.drones, use_dola=use_dola, plane_to_tag=plane_to_tag)

    # -- config map (the in-sim default) --------------------------------- #
    def _config_map(self) -> Dict[int, str]:
        return {tag: ip for ip, tag in self._units}

    # -- live Dola discovery (real day, opt-in) -------------------------- #
    def _dola_ips(self) -> Dict[int, str]:
        Dola = _resolve_dola()
        d = Dola()                                     # sim stub raises here already
        if hasattr(d, "start"):
            d.start()
        try:
            return dict(d.get_all_ips(self.listen_seconds))   # {plane_id: ip}
        finally:
            if hasattr(d, "stop"):
                d.stop()

    def resolve(self) -> Dict[int, str]:
        """Return {uwb_tag_id: ip}. Config-first; Dola only if opted in, with a
        transparent fall-back to config when Dola is unavailable/stubbed."""
        if self.use_dola:
            try:
                plane_ips = self._dola_ips()
                resolved = {self.plane_to_tag.get(pid, pid): ip
                            for pid, ip in plane_ips.items()}
                if resolved:
                    return resolved
            except NotImplementedError:
                pass                                   # sim stub → fall back to config
        return self._config_map()

    def resolve_ordered(self, log=None) -> Dict[int, str]:
        """Real-day (deck slide-6): Dola-discover, then map the discovered drones to the
        configured tag_ids **in order** (sorted by plane_id), regardless of the plane_id
        values. Logs the chosen ip↔tag pairing. Falls back to the config map if Dola is
        unavailable (the sim stub raises). Operator confirms the pairing on the day."""
        tag_ids = sorted(tag for _ip, tag in self._units)
        plane_ips: Dict[int, str] = {}
        try:
            plane_ips = self._dola_ips()               # {plane_id: ip}
        except NotImplementedError:
            plane_ips = {}
        if plane_ips:
            ordered = [ip for _pid, ip in sorted(plane_ips.items())]
            mapping = {tag_ids[i]: ordered[i]
                       for i in range(min(len(tag_ids), len(ordered)))}
        else:
            mapping = self._config_map()               # fallback (sim / no Dola)
        if log is not None:
            log(f"discovery: {len(plane_ips)} drone(s) found via Dola"
                if plane_ips else "discovery: Dola unavailable — using config IP map")
            for tag in sorted(mapping):
                log(f"  tag {tag}  <->  {mapping[tag]}")
        return mapping
