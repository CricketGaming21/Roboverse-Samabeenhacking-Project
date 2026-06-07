"""UWBParserThread — SIM drop-in for the real UWB serial parser.

Mission code does `from UWBParserThread import UWBParserThread` unchanged.
Signatures match the provided real file; behaviour per HULA_SIM_BUILD_PLAN §4.8:

  - get_tag_position(tag_id) -> (pos_x, pos_y, update_time): ARENA frame
    (x = north, y = east) in METRES, X-Y only — UWB gives no Z; altitude
    comes from get_altitude() (ToF). Unmapped tag_id -> (None, None, None).
  - Value = TRUE position + Gaussian noise (config uwb.noise_std_m). NO drift
    — unlike get_position(); that asymmetry is what lets the mission's
    UWB-correction loop be tested.
  - Samples refresh at uwb.rate_hz (SIM time). Between refreshes the SAME
    (value, update_time) pair is held; the tag dict is replaced wholesale
    each frame, exactly like the real parse_data.
  - update_time is wall-clock time.time() captured when the sample was
    produced, so mission freshness checks behave like hardware.
  - Origin-offset quirk preserved: the real code ACCEPTS x_origin/y_origin
    but origin_x/origin_y stay 0.0 and are never added to the output. Only
    config uwb.apply_origin_offset=true changes that.
  - Optional dropout (uwb.dropout_prob): a mapped tag occasionally vanishes
    for a frame to mimic NLOS/occlusion.

The first run() boots the shared sim world if nothing else has (§5.4).
Sim nicety: the thread is daemon=True (the real one is not) so a forgotten
stop() can't hang test runs — still call stop()/join() like on hardware.
"""

import threading
import time


class UWBParserThread(threading.Thread):

    def __init__(self, x_origin: float = 0.0, y_origin: float = 0.0,
                 serial_port=None, baud_rate: int = 921600) -> None:
        super().__init__(daemon=True)
        self.serial_port = serial_port if serial_port else self.detect_com_port()
        self.baud_rate = baud_rate
        self.data_lock = threading.Lock()
        self.tag_data = {}  # {tag_id: (pos_x, pos_y, update_time)}
        # Quirk matched to the provided real code: constructor args accepted,
        # but these stay 0.0 (and are what gets added to the output).
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.running = True
        self._x_origin_arg = float(x_origin)
        self._y_origin_arg = float(y_origin)

    def detect_com_port(self):
        """Sim: report a fake port; no hardware involved."""
        port = "/dev/ttySIM_UWB"
        print(f"Detected UWB Device on {port}")
        return port

    def run(self) -> None:
        """Sample registry ground truth at uwb.rate_hz (sim time) + noise."""
        import numpy as np
        from simcore.registry import get_registry

        try:
            reg = get_registry()  # boots the shared world on first UWB use
        except Exception as e:
            print(f"UWB sim error: failed to attach to sim world: {e}")
            self.running = False
            return
        cfg = reg.config
        rng = np.random.default_rng([cfg.meta.seed, 9000])
        noise_std = float(cfg.uwb.noise_std_m)
        dropout = float(cfg.uwb.dropout_prob)
        period_sim = 1.0 / float(cfg.uwb.rate_hz)
        if cfg.uwb.apply_origin_offset:  # default false == hardware behaviour
            self.origin_x = self._x_origin_arg
            self.origin_y = self._y_origin_arg
        # Wall sleep between refresh checks, scaled by the real-time factor.
        poll_s = max(0.001, min(
            period_sim / max(cfg.meta.real_time_factor, 1e-9) / 5.0, 0.05))

        next_due = reg.sim_time()
        while self.running:
            if not reg.is_alive():
                break
            now_sim = reg.sim_time()
            if now_sim >= next_due:
                try:
                    truth = reg.uwb_truth()
                except RuntimeError:
                    break  # sim shut down mid-read
                stamp = time.time()
                new_data = {}
                for tag_id, north, east in truth:
                    if dropout > 0.0 and rng.random() < dropout:
                        continue  # NLOS gap: tag absent this frame
                    nx, ny = rng.normal(0.0, noise_std, 2)
                    new_data[tag_id] = (north + nx + self.origin_x,
                                        east + ny + self.origin_y, stamp)
                with self.data_lock:
                    self.tag_data = new_data  # whole-dict replace, like parse_data
                next_due += period_sim
                if next_due < now_sim:  # fell behind (e.g. high rtf): resync
                    next_due = now_sim + period_sim
            time.sleep(poll_s)

    def get_tag_position(self, tag_id: int):
        """Returns the x, y position and update time of a specific tag."""
        with self.data_lock:
            return self.tag_data.get(tag_id, (None, None, None))

    def stop(self) -> None:
        """Stops the thread."""
        self.running = False
