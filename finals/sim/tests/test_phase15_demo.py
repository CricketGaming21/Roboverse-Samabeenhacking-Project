"""Phase 15 acceptance test — the full two-phase scenario demo (headless).

The demo must run BOTH phases on the authored map, score >=1 landing in
part 1 AND >=1 distinct rover id in part 2, keep rovers inert outside
AMBUSH, last the configured episode, and be a FIXED canned script — no
search / coordination / camera-driven control anywhere in it.
"""

import inspect

import scripts.scenario_demo as demo
from simcore.config import load_config


def test_full_demo_runs_both_phases_and_scores():
    cfg = load_config("sim_config.yaml")
    cfg.meta.real_time_factor = 10.0
    cfg.rovers.gimbal.enabled = False  # full-demo scoring; gimbal tested in phase36
    # Compact episode for the test (config-driven, semantics unchanged):
    cfg.scenario.episode_seconds = 55.0
    cfg.scenario.ambush_seconds = 45.0
    cfg.scenario.deploy_timeout_s = 30.0
    cfg.scenario.ambush_trigger.mode = "on_all_landed"
    cfg.scenario.ambush_trigger.delay_s = 1.0

    result = demo.run_scenario_demo(cfg)

    # Part 1: landings scored with accuracy on designated pads
    assert result["landing_score"] >= 1
    designated = {p.id for p in cfg.pads if p.valid and p.designated}
    for _drone, pad_id, err, t in result["landings"]:
        assert pad_id in designated
        assert err <= cfg.scoring.landing.tolerance_m
        assert t > 0

    # Part 2: distinct rover ids banked, never pads
    assert result["snapshot_score"] >= 1
    rover_ids = set(cfg.rovers.marker_ids)
    banked_ids = [b[0] for b in result["banked"]]
    assert set(banked_ids) <= rover_ids
    assert len(set(banked_ids)) == len(banked_ids)  # distinct

    # The run lasted the configured episode and finished
    assert result["final_phase"] == "done"
    assert result["sim_time"] >= cfg.scenario.episode_seconds - 0.5

    # Rovers moved ONLY during AMBUSH (inert + off-map in DEPLOY)
    deploy = [s for s in result["rover_samples"] if s[1] == "deploy"]
    ambush = [s for s in result["rover_samples"] if s[1] == "ambush"]
    assert deploy and ambush                       # both phases observed
    for _t, _ph, positions, in_arena in deploy:
        assert all(n < 0.0 for n, _e in positions)  # staged off-map
        assert not any(in_arena)
    for i in range(cfg.rovers.count):              # zero DEPLOY motion
        track = {tuple(s[2][i]) for s in deploy}
        assert len(track) == 1
    moved = any(s0[2] != s1[2] for s0, s1 in zip(ambush, ambush[1:]))
    assert moved                                   # convoy ran in AMBUSH

    # Phase 24: the demo is now cleanly paced — NO command thrash (every
    # command is blocking + sequential), so the monitor stays silent.
    assert sum(result["monitor"]["preemptions"].values()) == 0
    assert sum(result["monitor"]["rate_warnings"].values()) == 0


def test_demo_is_a_fixed_script_not_a_strategy():
    """Pin the canned-ness: fixed waypoint constants, and no way for the
    flight to see or react — no camera reads, no detection, no obstacle
    queries, no inter-drone coordination state."""
    assert isinstance(demo.OBSERVE_STATIONS, tuple)
    assert len(demo.OBSERVE_STATIONS) == 3
    for station in demo.OBSERVE_STATIONS:   # one fixed hover point per drone
        assert isinstance(station, tuple) and len(station) == 2
    assert isinstance(demo.DEPLOY_PAD_INDEX, tuple)
    assert isinstance(demo.LOCKON_PITCH_SEQ, tuple)   # fixed keyframes
    assert isinstance(demo.LOCKON_TARGETS, tuple)

    src = inspect.getsource(demo)
    for forbidden in ("cv2", "detectMarkers", "latest_frame", "to_rgb",
                      "create_video_stream", "VideoStream", "get_obstacles",
                      "get_position", "get_tag_position", "UWBParserThread"):
        assert forbidden not in src, (
            f"demo references {forbidden!r} — the canned flight must not "
            f"sense or react; that's mission territory")