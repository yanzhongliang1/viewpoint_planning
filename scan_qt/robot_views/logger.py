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
        lbl.setStyleSheet("font-weight: bold; color: #ccc;")

        btn_clear = QPushButton("清空")
        btn_clear.setFixedSize(50, 20)
        btn_clear.setStyleSheet("font-size: 10px; padding: 0;")
        btn_clear.clicked.connect(lambda: self.txt_log.clear())

        h_title.addWidget(lbl)
        h_title.addStretch()
        h_title.addWidget(btn_clear)
        layout.addLayout(h_title)

        # 文本框
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet(
            "background: #111; color: #eee; border: 1px solid #333; font-family: Consolas, monospace; font-size: 11px;")
        layout.addWidget(self.txt_log)

    def log(self, message: str, level: str = "INFO"):
        """向日志面板追加信息"""
        time_str = datetime.now().strftime("%H:%M:%S")

        color = "#eee"
        if level == "WARNING":
            color = "#ffaa00"
        elif level == "ERROR":
            color = "#ff4444"
        elif level == "SUCCESS":
            color = "#44ff44"

        html = f'<span style="color:#888;">[{time_str}]</span> <span style="color:{color};">[{level}]</span> {message}'
        self.txt_log.append(html)
        # 滚动到底部
        sb = self.txt_log.verticalScrollBar()
        sb.setValue(sb.maximum())
