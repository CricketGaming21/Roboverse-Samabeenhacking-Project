"""UWBParserThread — SIM drop-in for the real UWB serial parser.

Mission code does `from UWBParserThread import UWBParserThread` unchanged.
Signatures match the provided real file (reference/UWBParserThread.py);
behaviour contract is HULA_SIM_BUILD_PLAN.md §4.8:

  - get_tag_position(tag_id) -> (x, y, update_time) in METRES (arena frame,
    X-Y only, no Z), or (None, None, None) for unmapped tags / dropout.
  - Values = arena ground truth + Gaussian noise (uwb.noise_std_m). NO drift.
  - update_time is wall-clock time.time(); samples refresh at uwb.rate_hz and
    the same (value, timestamp) pair is held between refreshes.
  - Origin-offset quirk: x_origin/y_origin are accepted but NOT applied
    (matching the real code) unless config uwb.apply_origin_offset is true.

Phase 0: signatures only; behaviour lands in Phase 3.
"""

import threading


class UWBParserThread(threading.Thread):

    def __init__(self, x_origin: float = 0.0, y_origin: float = 0.0,
                 serial_port=None, baud_rate: int = 921600) -> None:
        raise NotImplementedError

    def detect_com_port(self):
        """Sim: return a fake port string; no hardware involved."""
        raise NotImplementedError

    def run(self) -> None:
        """Sim: pull ground truth from the registry, apply noise."""
        raise NotImplementedError

    def get_tag_position(self, tag_id: int):
        """Return (pos_x, pos_y, update_time) in metres, or (None, None, None)."""
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError
