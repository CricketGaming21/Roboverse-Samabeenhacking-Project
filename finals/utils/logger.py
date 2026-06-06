# =============================================================
# logger.py — shared logging utility
# Writes to both terminal and a timestamped log file
# =============================================================
import logging
import os
from datetime import datetime

LOG_DIR = os.path.join(os.path.dirname(__file__), "../claude_debug/logs")
os.makedirs(LOG_DIR, exist_ok=True)

def get_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    if logger.handlers:
        return logger

    fmt = "[%(asctime)s] %(levelname)s %(name)s: %(message)s"

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter(fmt, "%H:%M:%S"))

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    fh = logging.FileHandler(os.path.join(LOG_DIR, f"{name}_{timestamp}.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(fmt))

    logger.addHandler(ch)
    logger.addHandler(fh)
    return logger
