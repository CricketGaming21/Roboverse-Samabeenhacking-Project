"""Config layer — ALL sim tunables live here, with defaults, overridable by YAML.

Resolution order (later wins):
  1. The dataclass defaults below (mirror sim_config.yaml's documented values).
  2. A YAML file: explicit path arg > $HULA_SIM_CONFIG > ./sim_config.yaml.

Usage:
    from simcore.config import load_config
    cfg = load_config()              # default search
    cfg = load_config("other.yaml")  # explicit file
"""

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml

# Default YAML filename searched for in the current working directory
# (commands are always run from the project root — see CLAUDE.md).
DEFAULT_CONFIG_FILENAME = "sim_config.yaml"
ENV_VAR = "HULA_SIM_CONFIG"


# --------------------------------------------------------------------------- #
# Section dataclasses (defaults mirror sim_config.yaml)
# --------------------------------------------------------------------------- #

@dataclass
class MetaConfig:
    seed: int = 42                    # one seed drives arena gen, rover paths, noise, drift
    real_time_factor: float = 1.0     # 1.0 = real time; >1 runs faster


@dataclass
class PhysicsConfig:
    """Sim-thread stepping (defaults only — not surfaced in sim_config.yaml)."""
    dt_s: float = 1.0 / 240.0         # fixed physics timestep (sim time)
    gravity_mps2: float = -9.81
    max_catchup_steps: int = 24       # max physics steps per loop iteration when behind


@dataclass
class BodiesConfig:
    """Primitive collision-body sizes (defaults only — not in sim_config.yaml)."""
    drone_half_extents_m: list = field(
        default_factory=lambda: [0.09, 0.09, 0.04])   # ~18 cm square micro-drone
    rover_half_extents_m: list = field(
        default_factory=lambda: [0.16, 0.12, 0.135])  # ~RoboMaster S1 footprint


@dataclass
class ObstaclesConfig:
    count: int = 8
    footprint_min_m: float = 0.3
    footprint_max_m: float = 0.8
    height_min_m: float = 0.5
    height_max_m: float = 2.2
    min_clearance_m: float = 0.8
    keepclear_radius_m: float = 1.0


@dataclass
class AuthoredClusterSpec:
    """One crate cluster of the authored map: touching boxes around a centre."""
    center: list = field(default_factory=lambda: [5.0, 3.0])  # arena (north, east)
    boxes: int = 2
    height_m: list = field(default_factory=lambda: [0.4, 1.0])  # [min, max]


@dataclass
class ArchwayConfig:
    """Archway/tunnel feature: two pillars + a lintel to fly under/around."""
    corner: list = field(default_factory=lambda: [9.0, 5.0])  # arena (north, east)
    width_m: float = 1.0              # passage opening between the pillars
    height_m: float = 1.2             # clearance under the lintel


@dataclass
class AuthoredConfig:
    """Fixed layout matching the reference images (arena.layout: authored).
    Coordinates are APPROXIMATE (read off a low-res top-down) — adjust freely."""
    crate_m: float = 0.45             # crate footprint side; cluster boxes touch
    clusters: list = field(default_factory=lambda: [
        AuthoredClusterSpec(center=[5.0, 3.0], boxes=5, height_m=[0.4, 1.2]),
        AuthoredClusterSpec(center=[7.5, 4.2], boxes=2, height_m=[0.6, 1.0]),
        AuthoredClusterSpec(center=[4.6, 5.4], boxes=2, height_m=[0.4, 0.9]),
        AuthoredClusterSpec(center=[3.0, 3.5], boxes=2, height_m=[0.5, 1.1]),
    ])
    archway: ArchwayConfig = field(default_factory=ArchwayConfig)


@dataclass
class ArenaConfig:
    shape: str = "rectangle"
    layout: str = "authored"          # authored (matches images, DEFAULT) | procedural (seeded)
    length_m: float = 10.0            # extent along NORTH (x)
    width_m: float = 6.0              # extent along EAST  (y)
    height_m: float = 3.0
    wall_thickness_m: float = 0.1
    origin: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    yaw_deg: float = 0.0
    authored: AuthoredConfig = field(default_factory=AuthoredConfig)
    obstacles: ObstaclesConfig = field(default_factory=ObstaclesConfig)  # procedural only


@dataclass
class VelocityLevelsConfig:
    """m/s the kinematic model uses per pyhulax speed level."""
    SLOW: float = 0.3
    MEDIUM: float = 0.6
    ZOOM: float = 1.0
    TURBO: float = 1.5
    yaw_rate_dps: float = 60.0


@dataclass
class DroneSpec:
    ip: str = "10.0.0.11"
    uwb_tag_id: int = 0
    start: list = field(default_factory=lambda: [0.6, 1.5])  # arena (north, east) m
    heading_deg: float = 0.0          # 0 = facing +north


@dataclass
class BatteryConfig:
    start_pct: int = 100
    drain_pct_per_min: float = 8.0
    low_threshold_pct: int = 10


@dataclass
class DronesConfig:
    # Clustered at the SW entrance (where the convoy will enter), ~0.5 m apart.
    units: list = field(default_factory=lambda: [
        DroneSpec(ip="10.0.0.11", uwb_tag_id=0, start=[0.6, 1.1]),
        DroneSpec(ip="10.0.0.12", uwb_tag_id=1, start=[0.6, 0.6]),
        DroneSpec(ip="10.0.0.13", uwb_tag_id=2, start=[1.1, 0.6]),
    ])
    takeoff_height_cm: int = 100
    battery: BatteryConfig = field(default_factory=BatteryConfig)


@dataclass
class UWBConfig:
    rate_hz: float = 10.0
    noise_std_m: float = 0.05
    dropout_prob: float = 0.0
    apply_origin_offset: bool = False  # real code accepts origins but does NOT apply them
    x_origin: float = 0.0
    y_origin: float = 0.0


@dataclass
class PositionDriftConfig:
    enabled: bool = True
    random_walk_std_mps: float = 0.02


@dataclass
class BarrierRangeConfig:
    forward: float = 0.6
    back: float = 0.6
    left: float = 0.6
    right: float = 0.6
    down: float = 0.4


@dataclass
class BarrierSensorsConfig:
    range_m: BarrierRangeConfig = field(default_factory=BarrierRangeConfig)
    min_altitude_m: float = 0.35      # below this, barriers report clear


@dataclass
class CameraConfig:
    width: int = 640
    height: int = 480
    h_fov_deg: float = 70.0
    near_m: float = 0.05
    far_m: float = 25.0
    default_pitch_deg: float = 0.0    # 0 = forward, 90 = straight down
    fps: int = 30
    use_egl: bool = True              # load the EGL hardware-render plugin at boot
    mount_offset_m: float = 0.10      # front-mounted: lens sits ahead of centre


@dataclass
class ArucoConfig:
    dictionary: str = "DICT_6X6_250"
    pad_marker_size_m: float = 0.30
    rover_marker_size_m: float = 0.15


@dataclass
class PadSpec:
    id: int = 10
    north: float = 3.0
    east: float = 1.5
    valid: bool = True        # a legal landing zone (invalid = decoy/red marker)
    designated: bool = True   # one of the briefed "Land" targets this episode


@dataclass
class PatrolConfig:
    mode: str = "waypoint_random"
    speed_mps: float = 0.4
    waypoint_pause_s: float = 1.0
    bounds_north: list = field(default_factory=lambda: [1.0, 9.0])
    bounds_east: list = field(default_factory=lambda: [1.0, 5.0])


@dataclass
class RoversConfig:
    count: int = 5
    marker_ids: list = field(default_factory=lambda: [20, 21, 22, 23, 24])
    billboard_texture: str = "assets/robomaster.png"
    patrol: PatrolConfig = field(default_factory=PatrolConfig)


@dataclass
class AmbushTriggerConfig:
    """How the AMBUSH (convoy) phase starts — SCENARIO-owned, never
    mission-called (no pyhulax API can start it; see simcore/scenario.py)."""
    mode: str = "on_all_landed"   # on_all_landed | timed | manual_key
    delay_s: float = 3.0          # extra delay after the trigger event fires


@dataclass
class ScenarioConfig:
    """Two-phase episode: DEPLOY (land on pads) -> AMBUSH (convoy) -> DONE."""
    phases: str = "both"          # deploy | ambush | both
    episode_seconds: float = 180.0  # total run length (sim time)
    deploy_timeout_s: float = 90.0  # max part-1 time; then AMBUSH is forced
    ambush_seconds: float = 120.0   # how long the convoy phase runs
    entrance: list = field(default_factory=lambda: [0.5, 0.5])  # arena (n, e)
    ambush_trigger: AmbushTriggerConfig = field(
        default_factory=AmbushTriggerConfig)


@dataclass
class ScoringConfig:
    enabled: bool = True              # run the referee thread with the world
    mode: str = "auto"                # auto = sustained-visibility; explicit = capture hook
    min_marker_px: int = 40
    frame_margin_px: int = 8
    hold_frames: int = 5


@dataclass
class VizConfig:
    enabled: bool = True
    fps: int = 15
    show_camera_windows: bool = False


@dataclass
class MonitorConfig:
    max_cmd_rate_hz: float = 5.0
    warn_on_preempt: bool = True


@dataclass
class LoggingConfig:
    level: str = "INFO"
    file: str = "logs/sim.log"


@dataclass
class SimConfig:
    """Root config object — single source of truth for every tunable."""
    meta: MetaConfig = field(default_factory=MetaConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    bodies: BodiesConfig = field(default_factory=BodiesConfig)
    scenario: ScenarioConfig = field(default_factory=ScenarioConfig)
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    velocity_levels: VelocityLevelsConfig = field(default_factory=VelocityLevelsConfig)
    drones: DronesConfig = field(default_factory=DronesConfig)
    uwb: UWBConfig = field(default_factory=UWBConfig)
    position_drift: PositionDriftConfig = field(default_factory=PositionDriftConfig)
    barrier_sensors: BarrierSensorsConfig = field(default_factory=BarrierSensorsConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    aruco: ArucoConfig = field(default_factory=ArucoConfig)
    pads: list = field(default_factory=lambda: [
        PadSpec(id=10, north=8.5, east=3.0, valid=True, designated=True),
        PadSpec(id=11, north=5.5, east=4.8, valid=True, designated=True),
        PadSpec(id=12, north=2.0, east=4.5, valid=True, designated=True),
        PadSpec(id=13, north=5.5, east=1.2, valid=False, designated=False),
        PadSpec(id=14, north=2.0, east=1.5, valid=True, designated=False),
    ])
    rovers: RoversConfig = field(default_factory=RoversConfig)
    scoring: ScoringConfig = field(default_factory=ScoringConfig)
    viz: VizConfig = field(default_factory=VizConfig)
    monitor: MonitorConfig = field(default_factory=MonitorConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


# --------------------------------------------------------------------------- #
# YAML override machinery
# --------------------------------------------------------------------------- #

# Sections whose lists hold typed entries that need element-wise construction.
_LIST_ELEMENT_TYPES = {
    "units": DroneSpec,            # under drones:
    "pads": PadSpec,               # top level
    "clusters": AuthoredClusterSpec,  # under arena.authored:
}


def _apply_overrides(obj: Any, overrides: dict) -> None:
    """Recursively apply a dict of YAML overrides onto a dataclass tree."""
    for key, value in overrides.items():
        if not hasattr(obj, key):
            raise KeyError(
                f"Unknown config key '{key}' for {type(obj).__name__} "
                f"(check sim_config.yaml against simcore/config.py)"
            )
        current = getattr(obj, key)
        if isinstance(value, dict) and hasattr(current, "__dataclass_fields__"):
            _apply_overrides(current, value)
        elif isinstance(value, list) and key in _LIST_ELEMENT_TYPES:
            elem_type = _LIST_ELEMENT_TYPES[key]
            setattr(obj, key, [
                elem_type(**item) if isinstance(item, dict) else item
                for item in value
            ])
        else:
            setattr(obj, key, value)


def _resolve_config_path(path: Optional[str]) -> Optional[Path]:
    """Explicit arg > $HULA_SIM_CONFIG > ./sim_config.yaml (if present)."""
    if path is not None:
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"Config file not found: {p}")
        return p
    env = os.environ.get(ENV_VAR)
    if env:
        p = Path(env)
        if not p.is_file():
            raise FileNotFoundError(f"${ENV_VAR} points to a missing file: {p}")
        return p
    default = Path(DEFAULT_CONFIG_FILENAME)
    return default if default.is_file() else None


def load_config(path: Optional[str] = None) -> SimConfig:
    """Build a SimConfig from defaults, overlaid with the resolved YAML file.

    Returns pure defaults if no YAML file is found.
    """
    cfg = SimConfig()
    resolved = _resolve_config_path(path)
    if resolved is not None:
        with open(resolved, "r") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError(f"Top level of {resolved} must be a mapping")
        _apply_overrides(cfg, data)
    return cfg
