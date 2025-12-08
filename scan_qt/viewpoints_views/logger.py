# scan_qt\viewpoints_views\logger.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit
from datetime import datetime

class LogWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(2)

        # 标题栏
        h_title = QHBoxLayout()
        lbl = QLabel("系统日志 / Logs")
        lbl.setStyleSheet("font-weight: bold; color: #555;")

        btn_clear = QPushButton("清空")
        btn_clear.setFixedSize(50, 20)
        btn_clear.setStyleSheet("font-size: 10px;")
        btn_clear.clicked.connect(lambda: self.txt_log.clear())

        h_title.addWidget(lbl)
        h_title.addStretch()
        h_title.addWidget(btn_clear)
        layout.addLayout(h_title)

        # 文本框
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet("background: #ffffff; color: #333; border: 1px solid #dcdcdc; font-family: Consolas, monospace; font-size: 11px;")
        layout.addWidget(self.txt_log)

    def log(self, message: str, level: str = "INFO"):
        """向日志面板追加信息"""
        time_str = datetime.now().strftime("%H:%M:%S")

        color = "#333"  # 默认文字黑色
        level_color = "#555"

        if level == "WARNING":
            level_color = "#fa8c16"  # 橙色
        elif level == "ERROR":
            level_color = "#ff4d4f"  # 红色
        elif level == "SUCCESS":
            level_color = "#52c41a"  # 绿色
        elif level == "INFO":
            level_color = "#1890ff"  # 蓝色

        # 时间灰色，级别带色，内容深黑
        html = f'<span style="color:#999;">[{time_str}]</span> <span style="color:{level_color}; font-weight:bold;">[{level}]</span> <span style="color:{color}">{message}</span>'
        self.txt_log.append(html)
        # 滚动到底部
        sb = self.txt_log.verticalScrollBar()
        sb.setValue(sb.maximum())
