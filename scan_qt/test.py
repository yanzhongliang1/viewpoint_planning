# test_robot_ui.py（仅测试用）
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from scan_qt.views.robot_main_window import RobotMainWidget

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QMainWindow()
    win.setWindowTitle("Robot Comm Test")
    widget = RobotMainWidget()
    win.setCentralWidget(widget)
    win.resize(600, 400)
    win.show()
    sys.exit(app.exec_())
