"""Builds the PyBullet scene from an ArenaLayout + config.

Simulator INTERNAL — mission code must never import simcore.

build() MUST run on the sim thread (the registry calls it there) — PyBullet is
not thread-safe. Phase 1 bodies are simple coloured boxes with mass 0 (static):
no movement logic, sensors, or cameras yet. Drone/rover motion in later phases
is kinematic (resetBasePositionAndOrientation), so they stay massless.
"""

import math
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


_ROVER_RGBA = {
    "chassis": (0.93, 0.93, 0.95, 1.0),  # white shell, RoboMaster S1 look
    "wheel": (0.12, 0.12, 0.13, 1.0),    # dark Mecanum wheels
    "turret": (0.20, 0.22, 0.26, 1.0),   # gimbal/turret block
    "barrel": (0.10, 0.10, 0.12, 1.0),   # blaster barrel
}


def _make_rover(client, cfg, pose, marker_id) -> int:
    """Rover body shaped like a RoboMaster-style ground robot — chassis +
    four Mecanum wheels + a gimbal turret with a barrel — topped by this
    rover's UNIQUE ArUco marker (UV quad, faces up, decodes like the pads).
    Real geometry (not a flat billboard) so a mission-side YOLO model could
    plausibly detect it on a rendered frame.

    NOTE: YOLO itself is MISSION code run on the camera frames — this only
    makes the rovers visually detectable. A real model is validated on real
    footage and may need fine-tuning to close the sim-to-real gap."""
    hx, hy, hz = cfg.bodies.rover_half_extents_m
    pos = frames.arena_to_world(cfg, pose.north, pose.east, hz)
    orn = p.getQuaternionFromEuler(
        [0.0, 0.0, frames.heading_to_world_yaw_rad(cfg, pose.heading_deg)])
    # Collision stays the simple full box (motion/clearance unchanged).
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                                 physicsClientId=client)
    chassis_h = hz * 0.55
    chassis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[hx * 0.92, hy * 0.78, chassis_h],
        rgbaColor=_ROVER_RGBA["chassis"], physicsClientId=client)

    # Visual link shapes (mass 0, no collision) assembled around the chassis.
    link_vis, link_pos, link_orn = [], [], []
    wheel_r, wheel_w = hz * 0.85, hx * 0.34
    wheel = p.createVisualShape(
        p.GEOM_CYLINDER, radius=wheel_r, length=wheel_w,
        rgbaColor=_ROVER_RGBA["wheel"], physicsClientId=client)
    wx, wy = hx * 0.72, hy * 0.86
    wheel_q = list(p.getQuaternionFromEuler([math.pi / 2, 0, 0]))  # axle = y
    for sx in (-1, 1):
        for sy in (-1, 1):
            link_vis.append(wheel)
            link_pos.append([sx * wx, sy * wy, -hz + wheel_r * 0.7])
            link_orn.append(wheel_q)
    # gimbal turret + barrel — mounted FORWARD and kept BELOW the marker
    # plate so they never occlude the top fiducial.
    turret_h = hz * 0.28
    turret_z = chassis_h + turret_h
    turret = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[hx * 0.38, hy * 0.34, turret_h],
        rgbaColor=_ROVER_RGBA["turret"], physicsClientId=client)
    link_vis.append(turret)
    link_pos.append([0.0, hy * 0.28, turret_z])   # forward-mounted gimbal
    link_orn.append([0, 0, 0, 1])
    barrel = p.createVisualShape(
        p.GEOM_CYLINDER, radius=hz * 0.10, length=hy * 0.6,
        rgbaColor=_ROVER_RGBA["barrel"], physicsClientId=client)
    link_vis.append(barrel)
    link_pos.append([0.0, hy * 0.7, turret_z])
    link_orn.append(list(p.getQuaternionFromEuler([math.pi / 2, 0, 0])))
    # ArUco marker on the VERY top — the highest, widest surface, so nothing
    # else occludes it from a downward camera (the detectable face). LAST link.
    marker_side = cfg.aruco.rover_marker_size_m * aruco_assets.texture_scale()
    marker_quad = p.createVisualShape(
        p.GEOM_MESH, fileName=aruco_assets.ensure_quad_obj(marker_id),
        meshScale=[marker_side, marker_side, 1.0],
        rgbaColor=(1.0, 1.0, 1.0, 1.0), physicsClientId=client)
    link_vis.append(marker_quad)
    link_pos.append([0.0, 0.0, hz + 0.03])
    link_orn.append([0, 0, 0, 1])
    marker_link = len(link_vis) - 1

    n = len(link_vis)
    body = p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=chassis,
        basePosition=pos, baseOrientation=orn,
        linkMasses=[0] * n, linkCollisionShapeIndices=[-1] * n,
        linkVisualShapeIndices=link_vis,
        linkPositions=link_pos, linkOrientations=link_orn,
        linkInertialFramePositions=[[0, 0, 0]] * n,
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * n,
        linkParentIndices=[0] * n, linkJointTypes=[p.JOINT_FIXED] * n,
        linkJointAxis=[[0, 0, 1]] * n, physicsClientId=client)
    marker = p.loadTexture(aruco_assets.ensure_marker_png(cfg, marker_id),
                           physicsClientId=client)
    p.changeVisualShape(body, marker_link, textureUniqueId=marker,
                        physicsClientId=client)
    return body


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
        p.GEOM_MESH, fileName=aruco_assets.ensure_quad_obj(pad.id),
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
        _make_box(client, cfg, o.north, o.east, o.z0_m + o.height_m / 2,
                  o.half_n, o.half_e, o.height_m / 2, _RGBA["obstacle"])
        for o in layout.obstacles)

    drones = tuple(
        _make_agent_box(client, cfg, pose, cfg.bodies.drone_half_extents_m,
                        _RGBA["drone"])
        for pose in layout.drone_starts)

    marker_ids = list(cfg.rovers.marker_ids)
    n_rovers = len(layout.rover_starts)
    if len(marker_ids) < n_rovers:
        raise ValueError(f"rovers.marker_ids has {len(marker_ids)} ids for "
                         f"{n_rovers} rovers")
    if len(set(marker_ids[:n_rovers])) != n_rovers:
        raise ValueError(f"rover marker ids must be unique: {marker_ids}")
    rovers = tuple(
        _make_rover(client, cfg, pose, marker_ids[i])
        for i, pose in enumerate(layout.rover_starts))

    pads = tuple(_make_pad(client, cfg, pad) for pad in cfg.pads)

    return WorldBodies(floor=floor, walls=walls, obstacles=obstacles,
                       drones=drones, rovers=rovers, pads=pads)
