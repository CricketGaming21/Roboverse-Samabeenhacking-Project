"""Tests for scripts/hardware_check.py — the READ-ONLY bring-up check, against the fakes.

Verifies PASS/FAIL logic, that NO motion is ever commanded, the --drone filter, and that
the guarded telemetry init never arms.
"""

import pytest

from mission.config import load_config
from mission.runtime import sdk_compat
from scripts.hardware_check import DroneCheck, check_drone, main, run_check
from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb


def _silent(*_a, **_k):
    pass


# --------------------------------------------------------------------------- #
# PASS / FAIL logic
# --------------------------------------------------------------------------- #
def test_all_drones_pass(world):
    cfg = load_config()
    uwb = fuwb.FakeUWBParserThread(world=world)
    results = run_check(cfg, uwb=uwb, make_api=lambda: fpx.FakeDroneAPI(world), log=_silent)
    assert len(results) == 3
    assert all(r.passed for r in results)
    assert all(r.connected and r.telemetry_ok and r.uwb_ok for r in results)
    starts = cfg.starts_by_tag()
    for r in results:
        x, y, _ = r.uwb
        assert abs(x - starts[r.tag_id][0]) < 0.2 and abs(y - starts[r.tag_id][1]) < 0.2


def test_uwb_no_fix_is_fail(world):
    cfg = load_config()
    uwb = fuwb.FakeUWBParserThread(world=world)
    uwb.set_unseen(1, True)                      # tag 1 → (None, None, None)
    by_tag = {r.tag_id: r for r in run_check(
        cfg, uwb=uwb, make_api=lambda: fpx.FakeDroneAPI(world), log=_silent)}
    assert by_tag[1].connected and by_tag[1].telemetry_ok       # connected + telemetry fine
    assert by_tag[1].uwb_ok is False and by_tag[1].passed is False
    assert by_tag[0].passed and by_tag[2].passed                # the others still pass


def test_connect_failure_is_fail(world):
    cfg = load_config()
    uwb = fuwb.FakeUWBParserThread(world=world)

    class _BadConnect(fpx.FakeDroneAPI):
        def connect(self, ip):
            raise RuntimeError("no route to host")

    r = check_drone(_BadConnect(world), "10.0.0.11", 0, uwb, log=_silent)
    assert r.connected is False and r.passed is False and "no route" in r.error


# --------------------------------------------------------------------------- #
# READ-ONLY: never commands motion
# --------------------------------------------------------------------------- #
def test_never_commands_motion(world):
    cfg = load_config()
    uwb = fuwb.FakeUWBParserThread(world=world)
    created = []

    def mk():
        d = fpx.FakeDroneAPI(world)
        created.append(d)
        return d

    run_check(cfg, uwb=uwb, make_api=mk, log=_silent)
    assert len(created) == 3
    for d in created:
        assert d.manual_calls == 0               # never sent a stick frame
        assert d.alt_cm == 0.0                   # never took off
        assert d.yaw_deg == 0.0                  # never rotated


def test_prepare_telemetry_is_guarded_and_never_arms(make_drone, drone):
    sdk_compat.prepare_telemetry(drone)          # sim fake lacks the methods → no-op, no raise
    assert sdk_compat.heartbeat_running(drone) is False
    d = make_drone(0, real_like=True)
    sdk_compat.prepare_telemetry(d)
    assert "set_app_mode:1" in d.real_calls
    assert "send_app_heartbeat" in d.real_calls  # single handshake beat so telemetry flows
    assert "arm" not in d.real_calls             # READ-ONLY: must NOT arm
    assert "set_velocity_level" not in " ".join(d.real_calls)   # ...nor enter manual control
    assert sdk_compat.heartbeat_running(d) is False             # ...nor start the heartbeat thread


# --------------------------------------------------------------------------- #
# --drone filter + CLI
# --------------------------------------------------------------------------- #
def test_single_drone_filter(world):
    cfg = load_config()
    uwb = fuwb.FakeUWBParserThread(world=world)
    results = run_check(cfg, uwb=uwb, make_api=lambda: fpx.FakeDroneAPI(world),
                        only_ip="10.0.0.12", log=_silent)
    assert len(results) == 1
    assert results[0].ip == "10.0.0.12" and results[0].tag_id == 1


def test_unknown_ip_raises(world):
    cfg = load_config()
    uwb = fuwb.FakeUWBParserThread(world=world)
    with pytest.raises(SystemExit):
        run_check(cfg, uwb=uwb, make_api=lambda: fpx.FakeDroneAPI(world),
                  only_ip="9.9.9.9", log=_silent)


def test_main_exit_zero_when_all_pass(world, capsys):
    assert main([]) == 0
    out = capsys.readouterr().out
    assert "SUMMARY: 3/3 drone(s) PASS" in out


def test_main_single_drone(world, capsys):
    assert main(["--drone", "10.0.0.11"]) == 0
    assert "1/1 drone(s) PASS" in capsys.readouterr().out
