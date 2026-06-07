"""INTERNAL — routes DroneAPI calls to the sim registry (simcore).

Not part of the public pyhulax API. Mission code must never import this.
Implemented in Phase 2; Phase 0 ships the placeholder only.
"""


def get_registry():
    """Return the lazily-initialised shared sim world (simcore.registry)."""
    raise NotImplementedError("Phase 2: DroneAPI -> simcore registry bridge")
