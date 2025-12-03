# scan_qt/robot_views/robot_window.py
import numpy as np
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QMutex

# 导入各模块
from scan_qt.robot_views.robot_view import RobotView
from scan_qt.robot_views.robot_window_slots import RobotWindowSlots
from scan_qt.robot_views.robot_window_qss import QSS
from scan_qt.robot_views.robot_workers import MonitorThread
from scan_qt.robot_views.logger import setup_logging


class RobotWindow(RobotView, RobotWindowSlots):
    """
    主窗口类：继承自 View (界面) 和 Slots (逻辑)
    这种多重继承方式在 PyQt 开发中很常见，能有效分离 UI 和 逻辑。
    """

    def __init__(self):
        # 初始化 View (RobotView 继承自 QMainWindow)
        super().__init__()

        # 1. 基础设置
        self.setWindowTitle("CoppeliaSim UR5 Control Station (Pro)")
        self.resize(1600, 900)
        self.setStyleSheet(QSS)

        # 2. 初始化图表 (View 中的方法)
        self.create_plots()

        # 3. 初始化核心变量
        self.rc = None
        self.ik = None
        self.path = None
        self.programmatic_update = False

        # === 关键：创建互斥锁 ===
        # 这个锁将用于所有 ZMQ 通讯，防止多线程冲突导致 crash
        self.zmq_mutex = QMutex()

        # 4. 数据缓冲
        self.plot_data = np.zeros((7, 200))  # 7条曲线，200个点

        # 5. 日志
        self.log_handler = setup_logging()
        self.log_handler.log_signal.connect(self.append_log)

        # 6. 初始化监控线程 (传入 Mutex)
        self.monitor_thread = MonitorThread(None, self.zmq_mutex)
        self.monitor_thread.data_signal.connect(self.update_data)

        # 7. 绑定信号槽 (Slots 中的方法)
        self.setup_connections()

    def append_log(self, level, msg):
        """处理日志显示"""
        color = "#ffffff"
        if level == "INFO":
            color = "#69f0ae"
        elif level == "WARNING":
            color = "#ffd740"
        elif level == "ERROR":
            color = "#ff5252"

        # 自动滚动
        self.log_text.append(f'<span style="color:{color};">[{level}] {msg}</span>')
        sb = self.log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    def closeEvent(self, event):
        """窗口关闭事件"""
        self.close_app_cleanup()
        event.accept()
