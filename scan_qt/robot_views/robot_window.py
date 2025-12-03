# scan_qt/robot_views/robot_window.py
import numpy as np
import logging
from PyQt5.QtCore import QMutex

# 导入模块
from scan_qt.robot_views.robot_view import RobotView
from scan_qt.robot_views.robot_window_slots import RobotWindowSlots
from scan_qt.robot_views.robot_window_qss import QSS
from scan_qt.robot_views.robot_workers import MonitorThread


class RobotWindow(RobotView, RobotWindowSlots):
    def __init__(self):
        super().__init__()

        self.setStyleSheet(QSS)
        self.setWindowTitle("CoppeliaSim Control Center (Stable)")
        self.resize(1400, 900)

        # 1. 初始化全局互斥锁 (Core Mutex)
        self.zmq_mutex = QMutex()

        # 2. 初始化变量
        self.rc = None
        self.ik = None
        self.path = None
        self.plot_data = np.zeros((7, 200))
        self.programmatic_update = False

        # 3. 初始化线程 (传入锁)
        self.monitor_thread = MonitorThread(self.zmq_mutex)
        self.monitor_thread.data_signal.connect(self.update_data)

        # 4. 配置日志桥接
        self.setup_logging_bridge()

        # 5. 初始化图表和连接
        self.create_plots()
        self.setup_connections()

    def setup_logging_bridge(self):
        """将 Python logging 系统桥接到 UI 的 LogWidget"""

        # 定义一个简单的 Handler 类
        class BridgeHandler(logging.Handler):
            def __init__(self, widget):
                super().__init__()
                self.widget = widget

            def emit(self, record):
                msg = self.format(record)
                # 直接调用 LogWidget 的 log 方法
                # 注意：这里是在发出 log 的线程中调用的，
                # 但 PyQt 的 QTextEdit.append 是线程安全的 (大多情况下)，或者 LogWidget 内部可以用 Signal 优化
                # 鉴于您的 LogWidget 没有定义 Signal，这里假设直接调用。
                # 最稳妥的方式是在 LogWidget 里定义 pyqtSignal 并 connect
                self.widget.log(msg, record.levelname)

        # 获取 root logger
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)

        # 清理默认 handler，避免控制台重复输出
        # if logger.hasHandlers(): logger.handlers.clear()

        # 添加桥接 Handler
        bridge = BridgeHandler(self.log_widget)
        formatter = logging.Formatter('%(message)s')
        bridge.setFormatter(formatter)
        logger.addHandler(bridge)

        # --------------------------------------------------------
        # 日志桥接优化：确保线程安全
        # --------------------------------------------------------
        # 如果您使用的是标准 logging，建议定义一个 Signal 来跨线程更新 UI
        # 但由于您的 LogWidget 比较简单，且 QTextEdit 的 append 在某些 PyQt 版本是线程安全的，
        # 上面的直接调用可能也能工作。为了最稳妥，建议改为：

    def setup_logging_bridge_safe(self):
        """线程安全的日志桥接"""
        import logging

        # 1. 定义一个信号接收器 (必须继承 QObject)
        from PyQt5.QtCore import QObject, pyqtSignal

        class LogBridge(QObject):
            new_log = pyqtSignal(str, str)  # msg, level

        self.log_bridge = LogBridge()
        self.log_bridge.new_log.connect(self.log_widget.log)  # 连接到 UI 组件

        # 2. 定义 Handler
        class SafeHandler(logging.Handler):
            def __init__(self, signal_emitter):
                super().__init__()
                self.emitter = signal_emitter

            def emit(self, record):
                msg = self.format(record)
                # 发送信号 (线程安全)
                self.emitter.new_log.emit(msg, record.levelname)

        # 3. 配置 Logger
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)

        # 清除旧 Handler (防止重复)
        for h in logger.handlers:
            logger.removeHandler(h)

        h = SafeHandler(self.log_bridge)
        h.setFormatter(logging.Formatter('%(message)s'))
        logger.addHandler(h)

    def closeEvent(self, event):
        """窗口关闭事件：确保资源释放"""
        self.close_app_cleanup()
        event.accept()

    # 覆盖 __init__ 中的调用
    # 在 RobotWindow.__init__ 中，将 self.setup_logging_bridge() 改为 self.setup_logging_bridge_safe()
