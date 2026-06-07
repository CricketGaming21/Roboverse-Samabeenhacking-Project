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
class ObstaclesConfig:
    count: int = 8
    footprint_min_m: float = 0.3
    footprint_max_m: float = 0.8
    height_min_m: float = 0.5
    height_max_m: float = 2.2
    min_clearance_m: float = 0.8
    keepclear_radius_m: float = 1.0


@dataclass
class ArenaConfig:
    shape: str = "rectangle"
    length_m: float = 10.0            # extent along NORTH (x)
    width_m: float = 6.0              # extent along EAST  (y)
    height_m: float = 3.0
    wall_thickness_m: float = 0.1
    origin: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    yaw_deg: float = 0.0
    obstacles: ObstaclesConfig = field(default_factory=ObstaclesConfig)


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
    units: list = field(default_factory=lambda: [
        DroneSpec(ip="10.0.0.11", uwb_tag_id=0, start=[0.6, 1.5]),
        DroneSpec(ip="10.0.0.12", uwb_tag_id=1, start=[0.6, 3.0]),
        DroneSpec(ip="10.0.0.13", uwb_tag_id=2, start=[0.6, 4.5]),
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
class ScoringConfig:
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
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    velocity_levels: VelocityLevelsConfig = field(default_factory=VelocityLevelsConfig)
    drones: DronesConfig = field(default_factory=DronesConfig)
    uwb: UWBConfig = field(default_factory=UWBConfig)
    position_drift: PositionDriftConfig = field(default_factory=PositionDriftConfig)
    barrier_sensors: BarrierSensorsConfig = field(default_factory=BarrierSensorsConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    aruco: ArucoConfig = field(default_factory=ArucoConfig)
    pads: list = field(default_factory=lambda: [
        PadSpec(id=10, north=3.0, east=1.5),
        PadSpec(id=11, north=6.0, east=4.5),
        PadSpec(id=12, north=8.5, east=3.0),
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
    "units": DroneSpec,   # under drones:
    "pads": PadSpec,      # top level
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
