"""(Re)generate ArUco marker PNGs into assets/ (pad + rover ids, quiet zone).

Usage (from the project root): python -m scripts.gen_assets
"""

from simcore.aruco_assets import generate_all
from simcore.config import load_config


def main() -> None:
    cfg = load_config()
    paths = generate_all(cfg, force=True)
    print(f"dictionary: {cfg.aruco.dictionary}")
    for marker_id, path in sorted(paths.items()):
        kind = "pad" if marker_id in {p.id for p in cfg.pads} else "rover"
        print(f"  id {marker_id:3d} ({kind}):  {path}")
    print(f"{len(paths)} marker PNGs written.")


if __name__ == "__main__":
    main()
