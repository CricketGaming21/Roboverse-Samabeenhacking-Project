"""Typed mission config — pydantic load of `config/mission_config.yaml`.

**Rejects unknown keys** (every section is `extra="forbid"`) so a typo or a stale
field is caught at load, not in flight. Mirrors the sim's validated-config discipline.
Enforces the HARD speed cap (`speed.max_mps` must be > 0 and ≤ 0.5).
"""

from __future__ import annotations

from pathlib import Path
from typing import List

import yaml
from pydantic import BaseModel, ConfigDict, field_validator

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


class PlannerCfg(_Base):
    arena_truth_file: str
    inflate_m: float
    separation_min_m: float


class ArucoCfg(_Base):
    dictionary: str
    min_marker_px: int
    pad_ids: List[int]


class CameraCfg(_Base):
    width: int
    height: int
    h_fov_deg: float
    search_gimbal_deg: float
    read_gimbal_deg: float


class PadCfg(_Base):
    id: int
    north: float
    east: float
    valid: bool


class LandingCfg(_Base):
    hoop_tol_m: float
    confirm_pad_aruco: bool


class FailsafeCfg(_Base):
    battery_rtl_pct: int
    phase1_max_s: float
    phase2_budget_s: float
    lock_timeout_s: float


class EvaderCfg(_Base):
    secure_autonomous_first: bool
    contain_with_chokepoints: bool


class MissionConfig(_Base):
    meta: MetaCfg
    frame: FrameCfg
    speed: SpeedCfg
    uwb: UwbCfg
    planner: PlannerCfg
    aruco: ArucoCfg
    camera: CameraCfg
    pads: List[PadCfg]
    landing: LandingCfg
    failsafe: FailsafeCfg
    evader: EvaderCfg

    def valid_pads(self) -> List[PadCfg]:
        return [p for p in self.pads if p.valid]


def load_config(path=None) -> MissionConfig:
    """Load + validate the mission config. Unknown keys raise `ValidationError`."""
    p = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    data = yaml.safe_load(p.read_text())
    return MissionConfig(**data)
