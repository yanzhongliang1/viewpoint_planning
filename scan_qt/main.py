# scan_qt/main.py
from scan_qt.viewpoints_views.main_window import MainWindow
from PyQt5.QtWidgets import (
    QApplication
)
import sys

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())