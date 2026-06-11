#!/usr/bin/env python3
"""camera_check.py --ip <ip> — READ-ONLY: live video + our ArUco detector on real frames.

Validates that the real 20 cm markers decode on the live 640×480 feed and how big they read.
Prints decoded id + marker pixel size per frame. NO motion (the drone can be hand-held).

    python scripts/camera_check.py --ip 10.0.0.11

Public API only.
"""

from __future__ import annotations

import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from typing import Callable, List, Sequence, Tuple                  # noqa: E402

from mission.perception.aruco import confirm_with_aruco            # noqa: E402
from mission.perception.detector import frame_bgr                  # noqa: E402
from mission.perception.video import wait_for_first_frame          # noqa: E402
from mission.runtime import sdk_compat                             # noqa: E402


def scan_frames(drone, stream, *, frames: int = 30, dictionary: str = "DICT_6X6_250",
                rover_ids: Sequence[int] = (), warmup_timeout_s: float = 15.0,
                sleep: Callable = time.sleep,
                log: Callable = print) -> List[Tuple[int, int]]:
    """Enable video, run cv2.aruco on each frame, print id + side-px. Returns [(id, px), ...].
    Labels each id against the `rover_ids` ALLOW-LIST (no pad deny-list — a real rover is id 11).
    READ-ONLY — never commands motion."""
    allow = set(rover_ids)
    drone.set_video_stream(True)
    stream.start()
    # Real H.264 negotiation can take several seconds — wait for the first frame so the decode
    # loop below isn't 30 instant "(no frame yet)" giving up before the stream is even up.
    if wait_for_first_frame(stream, timeout_s=warmup_timeout_s, sleep=sleep) is None:
        log(f"  (no frame after {warmup_timeout_s:.0f}s warm-up — is the video stream up?)")
    seen: List[Tuple[int, int]] = []
    for i in range(max(1, frames)):
        frame = stream.latest_frame
        if frame is None:
            log(f"  frame {i:02d}: (no frame yet)")
            sleep(0.1)
            continue
        dets = confirm_with_aruco(frame_bgr(frame), dictionary)
        if not dets:
            log(f"  frame {i:02d}: no marker")
        for d in dets:
            side = max(d.bbox[2], d.bbox[3])
            kind = "ROVER (allow-listed)" if d.marker_id in allow else "id not in allow-list"
            log(f"  frame {i:02d}: id {d.marker_id}  {side}px  ({kind})")
            seen.append((d.marker_id, side))
        sleep(0.1)
    return seen


def main(argv=None) -> int:
    import argparse

    import pyhulax

    from mission.config import load_config

    ap = argparse.ArgumentParser(description="READ-ONLY live ArUco camera check (no motion).")
    ap.add_argument("--ip", required=True, help="drone IP")
    ap.add_argument("--frames", type=int, default=30)
    args = ap.parse_args(argv)

    cfg = load_config()
    cfg_aruco = cfg.aruco
    dictionary, rover_ids = cfg_aruco.dictionary, cfg_aruco.rover_ids
    d = pyhulax.DroneAPI()
    d.connect(args.ip)
    stream = d.create_video_stream()
    print("camera_check — READ-ONLY (no motion). Decoding live frames…\n")
    try:
        seen = scan_frames(d, stream, frames=args.frames, dictionary=dictionary,
                           rover_ids=rover_ids,
                           warmup_timeout_s=cfg.camera.video_warmup_timeout_s)
    finally:
        try:
            stream.stop()
        except Exception:
            pass
        try:
            sdk_compat.release(d)          # release the connection on exit (no held link)
        except Exception:
            pass
    ids = sorted({mid for mid, _ in seen})
    print(f"\n══ decoded ids this run: {ids} ══")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
