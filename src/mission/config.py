"""Typed mission config — pydantic load of `config/mission_config.yaml`.

**Rejects unknown keys** (every section is `extra="forbid"`) so a typo or a stale
field is caught at load, not in flight. Mirrors the sim's validated-config discipline.
Enforces the HARD speed cap (`speed.max_mps` must be > 0 and ≤ 0.5).
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator

DEFAULT_CONFIG_PATH = (Path(__file__).resolve().parents[2]
                       / "config" / "mission_config.yaml")


class _Base(BaseModel):
    model_config = ConfigDict(extra="forbid")


class MetaCfg(_Base):
    seed: int


class FrameCfg(_Base):
    yaw_offset_deg: float
    lock_yaw: bool
    invert_right: bool
    invert_forward: bool


class SpeedCfg(_Base):
    max_mps: float
    cruise_alt_m: float
    climb_mps: float
    arrive_tol_m: float
    ctrl_rate_hz: float
    kp_xy: float
    ki_xy: float
    kp_alt: float

    @field_validator("max_mps")
    @classmethod
    def _cap(cls, v: float) -> float:
        if not (0.0 < v <= 0.5):
            raise ValueError("speed.max_mps must be in (0, 0.5] — HARD competition cap")
        return v


class UwbCfg(_Base):
    noise_std_m: float
    hold_on_dropout: bool
    tag_ids: List[int]
    origin_x: float = 0.0       # per-cage UWB origin (real day); the latest UWBParserThread applies it
    origin_y: float = 0.0


class PlannerCfg(_Base):
    arena_truth_file: str
    inflate_m: float
    separation_min_m: float
    # Reduced footprint margin used ONLY for a pad's final approach/departure when the pad sits
    # inside the full inflated bubble of a thin obstacle (e.g. an arch post). Keeps pad ingress/
    # egress raw-clear without overflying. Defaults so older/real profiles load unchanged.
    pad_approach_inflate_m: float = 0.20


class ArucoCfg(_Base):
    dictionary: str             # sim DICT_6X6_250 / real DICT_7X7_1000 (detector is dict-agnostic)
    min_marker_px: int
    rover_ids: List[int]        # ALLOW-LIST: bank only these ids (no pad deny-list — a real rover is id 11)


class CameraCfg(_Base):
    width: int
    height: int
    h_fov_deg: float
    search_gimbal_deg: float
    read_gimbal_deg: float
    # R2 moving-gimbal search/read pitch (DOWN degrees: 0=forward, 90=nadir). A MODERATE
    # forward tilt held throughout — never steepened to nadir (the marker faces any yaw, so
    # nadir loses a side-facing marker). `use_nadir_search` restores the 90° baseline (a
    # selectable fallback). Defaults so older/real profiles load unchanged.
    search_pitch_deg: float = 52.0
    use_nadir_search: bool = False


class SearchCfg(_Base):
    """R2 phase-2 persistence: when a drone SEES a rover body but can't decode the marker
    (out of the gimbal cone), it holds on it until rotation brings the marker into the cone."""
    persist_timeout_s: float = 9.0      # hold past one full gimbal sweep (~8 s) before orbiting
    orbit_step_m: float = 0.6           # light lateral strafe to change bearing (≤ cruise, no overfly)
    max_orbits: int = 2                 # light-orbit attempts before giving up a target
    presence_min_area_px: int = 500     # min body-blob area to treat as a rover presence


class PadCfg(_Base):
    id: int
    north: float
    east: float
    valid: bool
    designated: bool = False        # [SYNC-WITH-SIM] one of the briefed "Land" targets


class LandingCfg(_Base):
    hoop_tol_m: float


class FailsafeCfg(_Base):
    battery_rtl_pct: int
    phase1_max_s: float
    phase2_budget_s: float
    lock_timeout_s: float


class EvaderCfg(_Base):
    secure_autonomous_first: bool
    contain_with_chokepoints: bool


class DroneUnitCfg(_Base):
    """One drone: control IP ↔ UWB tag id ↔ arena start (north, east) m.
    Sim: ip+start given (config-resolved discovery). Real: tag_id only — ip is
    Dola-discovered and the start is read from UWB at runtime."""
    tag_id: int
    ip: Optional[str] = None
    start: Optional[List[float]] = None


class DiscoveryCfg(_Base):
    use_dola: bool = False      # real day: broadcast-discover IPs via Dola (else config IPs)


class RealCfg(_Base):
    """Real-hardware-only init knobs. The sim's DroneAPI lacks the methods these drive
    (`send_app_heartbeat`, `set_velocity_level`, …), so they are no-ops on the sim — applied
    only on hardware via the `runtime/sdk_compat.py` hasattr guards. Defaults so the sim /
    older profiles load with no `real:` section."""
    heartbeat_hz: float = 10.0          # manual-control keep-alive rate (send_app_heartbeat thread)
    velocity_level: str = "MEDIUM"      # firmware band for the 0.5 m/s cap (SLOW/MEDIUM/ZOOM/TURBO
                                        # all clamp ≤0.5; MEDIUM is the usable max)

    @field_validator("heartbeat_hz")
    @classmethod
    def _positive_hz(cls, v: float) -> float:
        if v <= 0.0:
            raise ValueError("real.heartbeat_hz must be > 0")
        return v


class MissionConfig(_Base):
    meta: MetaCfg
    frame: FrameCfg
    speed: SpeedCfg
    uwb: UwbCfg
    drones: List[DroneUnitCfg]
    planner: PlannerCfg
    aruco: ArucoCfg
    camera: CameraCfg
    pads: List[PadCfg]
    landing: LandingCfg
    failsafe: FailsafeCfg
    evader: EvaderCfg
    search: SearchCfg = Field(default_factory=SearchCfg)   # R2 gimbal persistence (defaults if absent)
    real: RealCfg = Field(default_factory=RealCfg)         # real-hardware init (no-op on sim)
    discovery: Optional[DiscoveryCfg] = None    # real day; absent on sim → config-IP discovery

    def valid_pads(self) -> List[PadCfg]:
        return [p for p in self.pads if p.valid]

    def designated_pads(self) -> List[PadCfg]:
        """Valid AND designated pads — the only ones part-1 scoring counts."""
        return [p for p in self.pads if p.valid and p.designated]

    def drone_units(self) -> List[DroneUnitCfg]:
        return list(self.drones)

    def ip_for_tag(self) -> dict:
        return {u.tag_id: u.ip for u in self.drones if u.ip is not None}

    def starts_by_tag(self) -> dict:
        return {u.tag_id: (u.start[0], u.start[1])
                for u in self.drones if u.start is not None}

    def use_dola(self) -> bool:
        return bool(self.discovery and self.discovery.use_dola)


def load_config(path=None) -> MissionConfig:
    """Load + validate the mission config (sim/default). Unknown keys raise `ValidationError`."""
    p = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    return MissionConfig(**data)


REAL_CONFIG_PATH = DEFAULT_CONFIG_PATH.parent / "mission_real.yaml"


def load_real_config(path=None) -> MissionConfig:
    """Load the REAL-hardware profile (`config/mission_real.yaml`). The real profile writes
    pads as `{id: {x, y}}` + a `designated_pads` list (the deck's shape); this translates them
    to the internal `pads: [{id, north, east, valid, designated}]` so mission code is unchanged.
    `mission_config.yaml` (sim) is never touched."""
    p = Path(path) if path is not None else REAL_CONFIG_PATH
    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    designated = {int(i) for i in data.pop("designated_pads", [])}
    pads = data.get("pads")
    if isinstance(pads, dict):                  # {id: {x, y}} → list[PadCfg]
        data["pads"] = [{"id": int(pid), "north": float(pp["x"]), "east": float(pp["y"]),
                         "valid": True, "designated": int(pid) in designated}
                        for pid, pp in pads.items()]
    return MissionConfig(**data)
