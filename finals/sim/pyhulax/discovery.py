"""Dola discovery — sim drop-in for the real UDP-8668 broadcast listener.

In the sim, discovery returns the drones configured in sim_config.yaml
({plane_id: ip}) immediately (after a short fake delay), ignoring the
network entirely. Signatures match the real dola.py.
"""

from typing import Optional


class Dola:
    """Drone discovery listener (sim: backed by config, not UDP)."""

    def __init__(self, listen_ip: str = "0.0.0.0") -> None:
        raise NotImplementedError

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def clear(self) -> None:
        """Clear all cached aircraft."""
        raise NotImplementedError

    def get_ip_by_plane_id(self, plane_id: int,
                           listen_seconds: float = 5.0) -> Optional[str]:
        """Wait up to listen_seconds for a specific aircraft; IP string or None."""
        raise NotImplementedError

    def get_ips_by_plane_ids(self, plane_ids,
                             listen_seconds: float = 5.0) -> dict:
        """Return {plane_id: ip_or_None} for the requested aircraft."""
        raise NotImplementedError

    def get_all_ips(self, listen_seconds: float = 5.0) -> dict:
        """Return {plane_id: ip} for all drones the sim is configured with.

        In sim, returns the configured fake drones immediately (ignores
        listen_seconds beyond a short fake delay).
        """
        raise NotImplementedError

    def get_all_plane_info(self, listen_seconds: float = 0) -> dict:
        """Return all cached aircraft information."""
        raise NotImplementedError
