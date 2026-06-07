"""Two-phase episode state machine: DEPLOY -> AMBUSH -> DONE.

Simulator INTERNAL — mission code must never import simcore.

THE TRIGGER IS SCENARIO-OWNED. On the real day the organisers launch the
rover convoy; the drones never summon it. So the ambush starts from config
(scenario.ambush_trigger) and there is NO pyhulax/UWB API to start it —
mission code just flies, the rovers appear on the scenario's own schedule.
request_manual_trigger() is an OPERATOR hook (the 'a' key in --live), not a
mission API.

Owned and stepped by the registry ON THE SIM THREAD. The scenario never
flies a drone. In DEPLOY the rovers are parked OFF-MAP at the entrance
(inert: not stepped, not visible to any camera, not targets). In AMBUSH
they enter the arena and move (fixed convoy routes arrive in Phase 11 —
until then the existing patrol is the placeholder "active" behaviour).
At DONE everything stops moving.

Trigger modes:
- on_all_landed (default): every drone has taken off at least once and is
  now grounded (ground-truth landedness), then + delay_s.
- timed: the convoy enters delay_s after episode start.
- manual_key: an operator keypress in --live (+ delay_s).
DEPLOY also force-advances to AMBUSH at deploy_timeout_s — the convoy comes
on schedule whether or not the drones managed to land.

DONE when: episode_seconds elapse, the ambush window (ambush_seconds) ends,
or every rover marker id has been banked by the referee.
"""

from .log import get_logger

DEPLOY = "deploy"
AMBUSH = "ambush"
DONE = "done"

_PHASES_MODES = ("deploy", "ambush", "both")
_TRIGGER_MODES = ("on_all_landed", "timed", "manual_key")

# Off-map staging: rovers wait OUTSIDE the south wall by the entrance,
# one behind the other, until the ambush begins.
_STAGE_NORTH0_M = -0.6
_STAGE_PITCH_M = 0.6
_SCAN_CHECK_PERIOD_S = 0.25  # referee-poll throttle (sim time)


class Scenario:
    """Episode controller for one sim world (pass the SimRegistry)."""

    def __init__(self, registry):
        self._reg = registry
        self._cfg = registry.config.scenario
        if self._cfg.phases not in _PHASES_MODES:
            raise ValueError(f"unknown scenario.phases: {self._cfg.phases!r}")
        if self._cfg.ambush_trigger.mode not in _TRIGGER_MODES:
            raise ValueError("unknown scenario.ambush_trigger.mode: "
                             f"{self._cfg.ambush_trigger.mode!r}")
        self._log = get_logger("scenario", registry.config)
        self.phase = None              # set by on_boot
        self.ambush_started_at = None  # sim time AMBUSH began
        self._trigger_event_t = None   # sim time the trigger event occurred
        self._manual = False           # operator keypress flag (atomic bool)
        self._next_scan_check = 0.0
        self._rover_ids = [r.marker_id for r in registry.rovers]

    # ------------------------------------------------------------------ #
    # Lifecycle — SIM THREAD ONLY
    # ------------------------------------------------------------------ #

    def on_boot(self) -> None:
        """Enter the starting phase (called from registry._boot)."""
        if self._cfg.phases == "ambush":
            self._enter_ambush(self._reg.clock.now(), "phases=ambush")
        else:
            self._enter_deploy()

    def step(self, now: float) -> None:
        """Advance the state machine. Called every physics step (cheap)."""
        if self.phase == DEPLOY:
            if now >= self._cfg.episode_seconds:
                self._enter_done(now, "episode time elapsed")
                return
            if self._cfg.phases == "deploy":
                if now >= self._cfg.deploy_timeout_s:
                    self._enter_done(now, "deploy phase complete")
                return
            trig = self._cfg.ambush_trigger
            if self._trigger_event_t is None and self._trigger_fired():
                self._trigger_event_t = now
                self._log.info(
                    "ambush trigger fired (%s) at t=%.1fs — convoy enters "
                    "in %.1fs", trig.mode, now, trig.delay_s)
            if (self._trigger_event_t is not None
                    and now >= self._trigger_event_t + trig.delay_s):
                self._enter_ambush(now, f"trigger {trig.mode} + delay")
            elif now >= self._cfg.deploy_timeout_s:
                self._enter_ambush(now, "deploy timed out — convoy enters "
                                        "on schedule regardless")
        elif self.phase == AMBUSH:
            if now >= self._cfg.episode_seconds:
                self._enter_done(now, "episode time elapsed")
            elif now >= self.ambush_started_at + self._cfg.ambush_seconds:
                self._enter_done(now, "ambush window over")
            elif now >= self._next_scan_check:
                self._next_scan_check = now + _SCAN_CHECK_PERIOD_S
                if self._all_rovers_scanned():
                    self._enter_done(now, "all rovers scanned")

    # ------------------------------------------------------------------ #
    # Reads (any thread) + the OPERATOR hook
    # ------------------------------------------------------------------ #

    def rovers_active(self) -> bool:
        """Rovers move only during AMBUSH (registry gates stepping on this)."""
        return self.phase == AMBUSH

    def request_manual_trigger(self) -> None:
        """OPERATOR hook (the 'a' key in --live). NOT a mission API — on
        hardware the organisers start the convoy; nothing in pyhulax can."""
        self._manual = True

    # ------------------------------------------------------------------ #
    # Internals — SIM THREAD ONLY
    # ------------------------------------------------------------------ #

    def _enter_deploy(self) -> None:
        self.phase = DEPLOY
        entrance_e = float(self._cfg.entrance[1])
        for i, rover in enumerate(self._reg.rovers):
            rover.park_offmap(_STAGE_NORTH0_M - i * _STAGE_PITCH_M,
                              entrance_e)
        self._log.info(
            "phase DEPLOY: pads active, %d rovers staged off-map at the "
            "entrance (trigger: %s)", len(self._reg.rovers),
            self._cfg.ambush_trigger.mode)

    def _enter_ambush(self, now: float, reason: str) -> None:
        self.phase = AMBUSH
        self.ambush_started_at = now
        convoy = self._reg.config.rovers.motion == "convoy"
        entrance_e = float(self._cfg.entrance[1])
        for i, rover in enumerate(self._reg.rovers):
            if convoy and rover.in_arena:
                # phases=ambush boots skip DEPLOY parking — stage off-map
                # anyway so the convoy still ENTERS from the entrance.
                rover.park_offmap(_STAGE_NORTH0_M - i * _STAGE_PITCH_M,
                                  entrance_e)
            rover.activate(now)  # convoy: arm staggered entry; patrol: enter
        self._log.info(
            "phase AMBUSH at t=%.1fs (%s): %d rovers (%s) active for %.0fs",
            now, reason, len(self._reg.rovers),
            self._reg.config.rovers.motion, self._cfg.ambush_seconds)

    def _enter_done(self, now: float, reason: str) -> None:
        self.phase = DONE
        score = (self._reg.referee.score()
                 if self._reg.referee is not None else None)
        self._log.info("phase DONE at t=%.1fs (%s)%s", now, reason,
                       f" — score {score}" if score is not None else "")

    def _trigger_fired(self) -> bool:
        mode = self._cfg.ambush_trigger.mode
        if mode == "timed":
            return True  # event at episode start; AMBUSH at +delay_s
        if mode == "manual_key":
            return self._manual
        # on_all_landed: ground truth — every drone has flown and is now
        # grounded again (a drone that never took off does not count).
        drones = self._reg.drones
        return bool(drones) and all(
            d.takeoff_frame is not None and not d.flying for d in drones)

    def _all_rovers_scanned(self) -> bool:
        ref = self._reg.referee
        if ref is None or not self._rover_ids:
            return False
        return set(self._rover_ids) <= ref.banked_ids()
