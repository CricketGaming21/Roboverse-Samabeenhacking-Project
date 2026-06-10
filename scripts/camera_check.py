#!/usr/bin/env python3
"""camera_check.py --ip <ip> — READ-ONLY: live video + our ArUco detector on real frames.

Validates that the real 20 cm markers decode on the live 640×480 feed and how big they read.
Prints decoded id + marker pixel size per frame. NO motion (the drone can be hand-held).

    python scripts/camera_check.py --ip 10.0.0.11

Public API only.
"""

from __future__ import annotations

import time
from typing import Callable, List, Tuple

from mission.perception.aruco import confirm_with_aruco


def scan_frames(drone, stream, *, frames: int = 30, dictionary: str = "DICT_6X6_250",
                sleep: Callable = time.sleep, log: Callable = print) -> List[Tuple[int, int]]:
    """Enable video, run cv2.aruco on each frame, print id + side-px. Returns [(id, px), ...].
    READ-ONLY — never commands motion."""
    drone.set_video_stream(True)
    stream.start()
    seen: List[Tuple[int, int]] = []
    for i in range(max(1, frames)):
        frame = stream.latest_frame
        if frame is None:
            log(f"  frame {i:02d}: (no frame yet)")
            sleep(0.1)
            continue
        dets = confirm_with_aruco(frame.to_rgb(), dictionary)
        if not dets:
            log(f"  frame {i:02d}: no marker")
        for d in dets:
            side = max(d.bbox[2], d.bbox[3])
            kind = "pad/exclude" if 10 <= d.marker_id <= 14 else "ROVER"
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

    dictionary = load_config().aruco.dictionary
    d = pyhulax.DroneAPI()
    d.connect(args.ip)
    stream = d.create_video_stream()
    print("camera_check — READ-ONLY (no motion). Decoding live frames…\n")
    try:
        seen = scan_frames(d, stream, dictionary=dictionary)
    finally:
        try:
            stream.stop()
        except Exception:
            pass
    ids = sorted({mid for mid, _ in seen})
    print(f"\n══ decoded ids this run: {ids} ══")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
