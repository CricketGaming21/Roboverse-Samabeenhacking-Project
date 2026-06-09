"""Emit the authored arena to a plain ground-truth YAML for the mission side.

The mission loads the emitted file (plain data, like the organisers' Discord
coordinate file) WITHOUT importing simcore. Regenerate after editing
sim_config.yaml arena.authored.

Usage (from the project root):
    python -m scripts.emit_arena_truth                 # -> arena_truth.yaml
    python -m scripts.emit_arena_truth --out crates.yaml
"""

import argparse

from simcore.arena_truth import write_arena_truth
from simcore.config import load_config


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Emit the authored arena as plain "
                                 "ground-truth YAML for the mission.")
    ap.add_argument("--config", default=None,
                    help="config YAML (default: $HULA_SIM_CONFIG or ./sim_config.yaml)")
    ap.add_argument("--out", default="arena_truth.yaml",
                    help="output path (default: arena_truth.yaml)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    data = write_arena_truth(cfg, args.out)
    print(f"wrote {args.out}: {len(data['crates'])} crates, arena "
          f"{data['arena']['length_m']}x{data['arena']['width_m']} m, "
          f"archway at {data['archway']['corner']}")


if __name__ == "__main__":
    main()
