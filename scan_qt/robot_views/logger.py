# scan_qt/robot_views/logger.py
import logging
from PyQt5.QtCore import QObject, pyqtSignal


class QtLogHandler(logging.Handler, QObject):
    log_signal = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()
        QObject.__init__(self)

    def emit(self, record):
        msg = self.format(record)
        self.log_signal.emit(record.levelname, msg)


def setup_logging():
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    if logger.hasHandlers():
        logger.handlers.clear()

    formatter = logging.Formatter('[%(asctime)s] %(message)s', datefmt='%H:%M:%S')

    # Console
    ch = logging.StreamHandler()
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    # GUI
    qh = QtLogHandler()
    qh.setFormatter(formatter)
    logger.addHandler(qh)

    return qh
