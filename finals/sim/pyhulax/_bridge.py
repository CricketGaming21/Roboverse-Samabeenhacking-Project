"""INTERNAL — routes DroneAPI calls to the sim registry (simcore).

Not part of the public pyhulax API. Mission code must never import this.

simcore is imported lazily inside functions so `import pyhulax` stays
side-effect-free; the shared sim world boots on the first connect() (§5.4).
Blocking semantics: a goal is installed on the sim thread, completion is
decided there in SIM time; the caller just waits on the goal's event (the
periodic wakeup below is only a sim-thread liveness check, not a timeout).
"""

from .core import CommandResult
from .exceptions import PyhulaxError


def get_registry():
    """Return the shared sim world, booting it on first use."""
    from simcore.registry import get_registry as _get
    return _get()


def connect(ip: str):
    """Bind to the configured sim drone with this IP. Returns (registry, drone)."""
    reg = get_registry()
    drone = reg.drone_by_ip(ip)
    if drone is None:
        known = [d.spec.ip for d in reg.drones]
        raise PyhulaxError(
            f"no sim drone configured with ip {ip!r} (configured: {known})")
    reg.run_on_sim_thread(lambda: setattr(drone, "connected", True))
    return reg, drone


def submit(reg, drone, blocking: bool, kind: str, **params) -> CommandResult:
    """Install goal_<kind>(**params) on the sim thread; wait if blocking.

    Validation (NotReady/LowBattery) runs on the sim thread and propagates
    to the caller here. blocking=False returns as soon as the goal is
    accepted; the motion continues on the sim thread.
    """
    goal = reg.run_on_sim_thread(
        lambda: getattr(drone, f"goal_{kind}")(**params))
    if not blocking:
        return CommandResult(True, f"{kind} accepted")
    while not goal.done.wait(0.25):
        if not reg.is_alive():
            raise RuntimeError(
                f"sim thread stopped while waiting for {kind} to complete")
    return goal.result


def read_battery(reg, drone) -> int:
    return reg.run_on_sim_thread(lambda: int(round(drone.battery_pct)))


def read_position(reg, drone):
    return reg.run_on_sim_thread(drone.telemetry_position)


def read_orientation(reg, drone):
    return reg.run_on_sim_thread(drone.telemetry_orientation)


def read_altitude(reg, drone) -> float:
    return reg.run_on_sim_thread(drone.telemetry_altitude)


def read_state(reg, drone):
    return reg.run_on_sim_thread(drone.telemetry_state)


def read_obstacles(reg, drone):
    return reg.run_on_sim_thread(drone.sense_obstacles)


def read_status(reg, drone) -> int:
    return reg.run_on_sim_thread(drone.status_bitmask)


def set_barrier_mode(reg, drone, enabled: bool) -> CommandResult:
    reg.run_on_sim_thread(lambda: drone.set_barrier_mode(enabled))
    return CommandResult(True, f"barrier mode "
                               f"{'enabled' if enabled else 'disabled'}")


def set_avoidance(reg, drone, direction, distance_cm, barrier_mask) -> CommandResult:
    reg.run_on_sim_thread(
        lambda: drone.set_avoidance_rule(direction, distance_cm, barrier_mask))
    armed = distance_cm and float(distance_cm) > 0
    return CommandResult(True, "avoidance rule armed" if armed
                               else "avoidance rule disarmed")
