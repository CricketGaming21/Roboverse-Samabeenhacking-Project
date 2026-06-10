"""Real-hardware config profile (config/mission_real.yaml) + open-cage arena.

Verifies the real profile translates to the internal schema, the safety values are baked,
and the SIM config path is untouched (backward-compatible schema extensions).
"""

from pathlib import Path

import pytest

from mission.config import (DiscoveryCfg, load_config, load_real_config)
from mission.planner.arena import load_arena

_ROOT = Path(__file__).resolve().parent.parent


# --------------------------------------------------------------------------- #
# real profile
# --------------------------------------------------------------------------- #
def test_real_profile_loads_and_translates_pads():
    cfg = load_real_config()
    coords = {p.id: (p.north, p.east) for p in cfg.pads}
    assert coords == {11: (1.35, 4.40), 45: (1.30, 7.85), 51: (4.40, 4.40),
                      67: (1.95, 8.70), 101: (4.40, 7.85)}
    assert {p.id for p in cfg.designated_pads()} == {11, 51, 101}
    assert all(p.valid for p in cfg.pads)                # operator sets the real valid set on the day


def test_real_profile_discovery_and_uwb_origin():
    cfg = load_real_config()
    assert cfg.use_dola() is True
    assert isinstance(cfg.discovery, DiscoveryCfg)
    assert cfg.uwb.origin_x == 0.0 and cfg.uwb.origin_y == 0.0


def test_real_profile_drones_are_tag_only():
    cfg = load_real_config()
    assert [u.tag_id for u in cfg.drones] == [0, 1, 2]
    assert cfg.starts_by_tag() == {}                     # starts read from UWB at runtime
    assert cfg.ip_for_tag() == {}                        # IPs Dola-discovered, not configured


def test_real_profile_safety_values():
    cfg = load_real_config()
    assert cfg.speed.max_mps == 0.5
    assert cfg.speed.cruise_alt_m == pytest.approx(1.1)
    assert cfg.landing.hoop_tol_m == pytest.approx(0.20)
    assert cfg.uwb.hold_on_dropout is True
    assert cfg.failsafe.battery_rtl_pct == 20


def test_real_arena_is_open_cage():
    arena = load_arena(_ROOT / "config" / "arena_real.yaml")
    assert arena.crates == []                            # no crates → straight UWB routes
    assert arena.length_m == 11.0 and arena.width_m == 11.0


# --------------------------------------------------------------------------- #
# sim config untouched (backward-compatible)
# --------------------------------------------------------------------------- #
def test_sim_config_still_loads_with_defaults():
    cfg = load_config()                                  # mission_config.yaml
    assert cfg.use_dola() is False                       # no discovery section
    assert cfg.discovery is None
    assert cfg.uwb.origin_x == 0.0 and cfg.uwb.origin_y == 0.0   # defaulted
    assert len(cfg.starts_by_tag()) == 3                 # sim drones keep ip+start
    assert cfg.ip_for_tag() == {0: "10.0.0.11", 1: "10.0.0.12", 2: "10.0.0.13"}
