"""5-direction barrier ray-casts (IR/ToF-like) + downward ToF altitude.

Simulator INTERNAL — mission code must never import simcore.

COARSE BY DESIGN (§4.4): the barrier surface is five BOOLEANS — no distances,
no point cloud, no map, no planner. The only range that exists is each ray's
trigger length from config.barrier_sensors.range_m. Horizontal rays are
body-relative to the CURRENT heading; there is no UP sensor.

MIN-ALTITUDE GATE: below config.barrier_sensors.min_altitude_m (ToF altitude),
all barriers report clear — like the real sensors, which need ~0.35 m to work.

ALL functions here are SIM-THREAD ONLY (they call pybullet).
"""

import pybullet as p

from pyhulax.core import Direction, Obstacles

from . import frames

# Rays start just outside the drone's hull so they never hit its own body
# (drone half extents are 0.09 x/y, 0.04 z).
_RAY_START_OFFSET_M = 0.10
_DOWN_START_OFFSET_M = 0.05
_TOF_MAX_RANGE_M = 10.0

_HORIZONTAL = (
    ("forward", Direction.FORWARD),
    ("back", Direction.BACK),
    ("left", Direction.LEFT),
    ("right", Direction.RIGHT),
)


def altitude_cm(client, cfg, drone) -> float:
    """Downward ToF: distance (cm) from the drone CENTRE to whatever is below
    — floor or obstacle top — via a real ray-cast."""
    x, y, z = drone.pos
    hit = p.rayTest((x, y, z - _DOWN_START_OFFSET_M),
                    (x, y, z - _TOF_MAX_RANGE_M),
                    physicsClientId=client)[0]
    if hit[0] >= 0 and hit[0] != drone.body_id:
        return (z - hit[3][2]) * 100.0
    return (z - float(cfg.arena.origin[2])) * 100.0  # fallback: flat floor


def barrier_flags(client, cfg, drone) -> Obstacles:
    """Fresh 5-direction barrier read for one drone (gated by min altitude)."""
    sens = cfg.barrier_sensors
    if altitude_cm(client, cfg, drone) / 100.0 < sens.min_altitude_m:
        return Obstacles()  # gate: sensors inert near the ground

    x, y, z = drone.pos
    starts, ends, names = [], [], []
    for name, direction in _HORIZONTAL:
        dx, dy, _ = frames.body_direction_to_world(drone.yaw, direction)
        reach = float(getattr(sens.range_m, name))
        starts.append((x + dx * _RAY_START_OFFSET_M,
                       y + dy * _RAY_START_OFFSET_M, z))
        ends.append((x + dx * reach, y + dy * reach, z))
        names.append(name)
    # Down barrier: short trigger ray (distinct from the long ToF ray above).
    starts.append((x, y, z - _DOWN_START_OFFSET_M))
    ends.append((x, y, z - float(sens.range_m.down)))
    names.append("down")

    hits = p.rayTestBatch(starts, ends, physicsClientId=client)
    return Obstacles(**{
        name: bool(hit[0] >= 0 and hit[0] != drone.body_id)
        for name, hit in zip(names, hits)
    })


def bitmask(obstacles: Obstacles) -> int:
    """Status bits per §4.4: 0=forward 1=back 2=left 3=right 4=down."""
    return (int(obstacles.forward)
            | int(obstacles.back) << 1
            | int(obstacles.left) << 2
            | int(obstacles.right) << 3
            | int(obstacles.down) << 4)
