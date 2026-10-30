"""Process-wide logging for nimo_arm_gui (stdlib logging + optional Qt UI sink)."""

from __future__ import annotations

import logging
import os
import sys
from typing import Optional

from PyQt5.QtCore import QObject, pyqtSignal

_LOG_FORMAT = "%(asctime)s %(levelname)s [%(name)s] %(message)s"
_LOG_DATEFMT = "%H:%M:%S"
_CONFIGURED = False


def get_logger(name: Optional[str] = None) -> logging.Logger:
    if name is None or name == "nimo_arm_gui":
        return logging.getLogger("nimo_arm_gui")
    if name.startswith("nimo_arm_gui."):
        return logging.getLogger(name)
    return logging.getLogger(f"nimo_arm_gui.{name}")


def configure_logging(level: Optional[str] = None) -> None:
    """Idempotent: configure nimo_arm_gui logger + stderr handler."""
    global _CONFIGURED
    raw = (level or os.environ.get("LOG_LEVEL") or "INFO").upper()
    lvl = getattr(logging, raw, logging.INFO)

    root = logging.getLogger("nimo_arm_gui")
    root.setLevel(lvl)
    root.propagate = False

    # Replace handlers on reconfigure (tests / restart).
    for h in list(root.handlers):
        root.removeHandler(h)
        h.close()

    sh = logging.StreamHandler(sys.stderr)
    sh.setLevel(lvl)
    sh.setFormatter(logging.Formatter(_LOG_FORMAT, datefmt=_LOG_DATEFMT))
    root.addHandler(sh)
    _CONFIGURED = True


class QtLogHandler(logging.Handler, QObject):
    """Bridge logging records into the Qt UI thread via a signal."""

    sig_record = pyqtSignal(str)

    def __init__(self, parent: Optional[QObject] = None) -> None:
        logging.Handler.__init__(self)
        QObject.__init__(self, parent)
        self.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s %(message)s", datefmt=_LOG_DATEFMT))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            self.sig_record.emit(msg)
        except Exception:
            self.handleError(record)


def attach_qt_handler(handler: QtLogHandler) -> None:
    root = logging.getLogger("nimo_arm_gui")
    # Avoid duplicate UI handlers.
    for h in list(root.handlers):
        if isinstance(h, QtLogHandler):
            root.removeHandler(h)
            h.close()
    root.addHandler(handler)


def detach_qt_handlers() -> None:
    root = logging.getLogger("nimo_arm_gui")
    for h in list(root.handlers):
        if isinstance(h, QtLogHandler):
            root.removeHandler(h)
            h.close()
