"""Video-stream helpers shared by the mission and the bring-up scripts.

The only public API we touch is `stream.latest_frame` (the pyhulax `VideoStream` surface,
mirrored by the fake). Nothing here is sim-specific.
"""

from __future__ import annotations

import time
from typing import Callable, Optional


def wait_for_first_frame(stream, timeout_s: float = 15.0, *, poll_s: float = 0.1,
                         sleep: Callable = time.sleep):
    """Poll `stream.latest_frame` until a non-None frame arrives, or `timeout_s` elapses.

    On real hardware the first frame after `set_video_stream(True)` + `stream.start()` can take
    **several seconds** (H.264 stream negotiation); the stream yields `None` until then. Code that
    samples a couple of frames and bails would mistake that warm-up for "nothing in view".

    The poll is bounded by a fixed iteration count (≈ `timeout_s / poll_s`), so it ALWAYS
    terminates — never an unbounded wait, regardless of the injected `sleep` (honours the
    project's no-unbounded-loop invariant). Returns the first non-None frame, or `None` if the
    stream never warmed up within the budget (the caller treats that as "no video").
    """
    polls = max(1, int(round(timeout_s / poll_s))) if poll_s > 0 else 1
    frame: Optional[object] = stream.latest_frame
    for _ in range(polls):
        if frame is not None:
            return frame
        sleep(poll_s)
        frame = stream.latest_frame
    return frame
