"""Builds the PyBullet scene from an ArenaLayout + config.

Simulator INTERNAL — mission code must never import simcore.

build() MUST run on the sim thread (the registry calls it there) — PyBullet is
not thread-safe. Phase 1 bodies are simple coloured boxes with mass 0 (static):
no movement logic, sensors, or cameras yet. Drone/rover motion in later phases
is kinematic (resetBasePositionAndOrientation), so they stay massless.
"""

from dataclasses import dataclass

import pybullet as p

from . import aruco_assets, frames

_FLOOR_HALF_H = 0.05  # thin static slab; its top face is exactly z = 0
_PAD_HALF_H = 0.005   # landing pads: 1 cm slabs lying flat on the floor

_RGBA = {
    "floor": (0.85, 0.85, 0.88, 1.0),
    "wall": (0.45, 0.45, 0.50, 1.0),
    "obstacle": (0.85, 0.50, 0.15, 1.0),
    "drone": (0.15, 0.35, 0.90, 1.0),
    "rover": (0.80, 0.10, 0.10, 1.0),
    "pad": (1.0, 1.0, 1.0, 1.0),  # white: texture colours pass through as-is
}


@dataclass(frozen=True)
class WorldBodies:
    """PyBullet body ids of everything in the scene, by kind."""
    floor: int
    walls: tuple
    obstacles: tuple
    drones: tuple
    rovers: tuple
    pads: tuple

    @property
    def total(self) -> int:
        return 1 + len(self.walls) + len(self.obstacles) + len(self.drones) \
            + len(self.rovers) + len(self.pads)


def _make_box(client, cfg, north, east, z_center, half_n, half_e, half_h,
              rgba, yaw_world_rad=None):
    """Create one static box. Local axes: +x = arena east, +y = arena north
    (at zero body yaw), so halfExtents = [half_e, half_n, half_h]."""
    if yaw_world_rad is None:
        yaw_world_rad = frames.arena_yaw_world_rad(cfg)
    pos = frames.arena_to_world(cfg, north, east, z_center)
    orn = p.getQuaternionFromEuler([0.0, 0.0, yaw_world_rad])
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_e, half_n, half_h],
                                 physicsClientId=client)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_e, half_n, half_h],
                              rgbaColor=rgba, physicsClientId=client)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                             baseVisualShapeIndex=vis, basePosition=pos,
                             baseOrientation=orn, physicsClientId=client)


def _make_agent_box(client, cfg, pose, half_extents, rgba):
    """Drone/rover body: local +x is the nose, oriented by heading_deg."""
    hx, hy, hz = half_extents
    pos = frames.arena_to_world(cfg, pose.north, pose.east, hz)
    yaw = frames.heading_to_world_yaw_rad(cfg, pose.heading_deg)
    orn = p.getQuaternionFromEuler([0.0, 0.0, yaw])
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                                 physicsClientId=client)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                              rgbaColor=rgba, physicsClientId=client)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                             baseVisualShapeIndex=vis, basePosition=pos,
                             baseOrientation=orn, physicsClientId=client)


def _make_pad(client, cfg, pad) -> int:
    """ArUco landing pad: textured slab flat on the floor, facing up, at the
    CONFIGURED arena coordinates. Pad side is sized so the printed marker
    (incl. black border) measures exactly aruco.pad_marker_size_m. The visual
    is a UV-mapped quad (GEOM_BOX auto-UVs would crop the marker)."""
    side = cfg.aruco.pad_marker_size_m * aruco_assets.texture_scale()
    pos = frames.arena_to_world(cfg, pad.north, pad.east, _PAD_HALF_H)
    orn = p.getQuaternionFromEuler([0.0, 0.0, frames.arena_yaw_world_rad(cfg)])
    col = p.createCollisionShape(p.GEOM_BOX,
                                 halfExtents=[side / 2, side / 2, _PAD_HALF_H],
                                 physicsClientId=client)
    vis = p.createVisualShape(
        p.GEOM_MESH, fileName=aruco_assets.ensure_quad_obj(),
        meshScale=[side, side, 1.0], rgbaColor=_RGBA["pad"],
        visualFramePosition=[0.0, 0.0, _PAD_HALF_H + 0.001],  # just atop slab
        physicsClientId=client)
    body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                             baseVisualShapeIndex=vis, basePosition=pos,
                             baseOrientation=orn, physicsClientId=client)
    tex = p.loadTexture(aruco_assets.ensure_marker_png(cfg, pad.id),
                        physicsClientId=client)
    p.changeVisualShape(body, -1, textureUniqueId=tex, physicsClientId=client)
    return body


def build(client: int, cfg, layout) -> WorldBodies:
    """Create floor, walls, obstacles, pads, drones, rovers. Sim thread only."""
    L, W, t = layout.length_m, layout.width_m, layout.wall_thickness_m

    floor = _make_box(client, cfg, L / 2, W / 2, -_FLOOR_HALF_H,
                      L / 2 + t, W / 2 + t, _FLOOR_HALF_H, _RGBA["floor"])

    walls = tuple(
        _make_box(client, cfg, w.north, w.east, w.half_h,
                  w.half_n, w.half_e, w.half_h, _RGBA["wall"])
        for w in layout.walls)

    obstacles = tuple(
        _make_box(client, cfg, o.north, o.east, o.height_m / 2,
                  o.half_n, o.half_e, o.height_m / 2, _RGBA["obstacle"])
        for o in layout.obstacles)

    drones = tuple(
        _make_agent_box(client, cfg, pose, cfg.bodies.drone_half_extents_m,
                        _RGBA["drone"])
        for pose in layout.drone_starts)

    rovers = tuple(
        _make_agent_box(client, cfg, pose, cfg.bodies.rover_half_extents_m,
                        _RGBA["rover"])
        for pose in layout.rover_starts)

    pads = tuple(_make_pad(client, cfg, pad) for pad in cfg.pads)

    return WorldBodies(floor=floor, walls=walls, obstacles=obstacles,
                       drones=drones, rovers=rovers, pads=pads)
