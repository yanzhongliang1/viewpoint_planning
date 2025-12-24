# scan_qt/robot_main.py
import sys
import os

# 确保 Python 能找到 scan_qt 包
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from PyQt5.QtWidgets import QApplication
from scan_qt.robot_views.robot_window import RobotWindow


def main():
    # 启用高分屏支持 (可选)
    # os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    # QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)

    app = QApplication(sys.argv)

    # 实例化主窗口
    window = RobotWindow()
    window.setup_logging_bridge_safe()  # 使用线程安全的日志
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
