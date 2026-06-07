"""Sim logger (console + file), configured from config.logging.

Simulator INTERNAL — mission code must never import simcore.
"""

import logging
import threading
from pathlib import Path

_ROOT_NAME = "hulasim"
_setup_lock = threading.Lock()
_configured = False


def setup_logging(cfg=None) -> None:
    """Configure the 'hulasim' root logger once (console + optional file)."""
    global _configured
    with _setup_lock:
        if _configured:
            return
        level = cfg.logging.level if cfg else "INFO"
        root = logging.getLogger(_ROOT_NAME)
        root.setLevel(level)
        root.propagate = False
        fmt = logging.Formatter(
            "%(asctime)s %(name)s %(levelname)s: %(message)s", "%H:%M:%S")

        console = logging.StreamHandler()
        console.setFormatter(fmt)
        root.addHandler(console)

        log_file = cfg.logging.file if cfg else None
        if log_file:
            try:
                Path(log_file).parent.mkdir(parents=True, exist_ok=True)
                fh = logging.FileHandler(log_file)
                fh.setFormatter(fmt)
                root.addHandler(fh)
            except OSError as e:  # never let logging break the sim
                root.warning("could not open log file %s: %s", log_file, e)
        _configured = True


def get_logger(name: str, cfg=None) -> logging.Logger:
    """Return a child logger ('hulasim.<name>'), configuring on first use."""
    setup_logging(cfg)
    return logging.getLogger(f"{_ROOT_NAME}.{name}")
