# scan_qt/views/robot_main_window_qss.py
"""
界面样式表
工业风格设计
"""

ROBOT_WINDOW_QSS = """
/* ==================== 全局样式 ==================== */
QWidget {
    background-color: #2b2b2b;
    color: #e0e0e0;
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
    font-size: 9pt;
}

/* ==================== 分组框 ==================== */
QGroupBox {
    border: 2px solid #3d3d3d;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 8px;
    font-weight: bold;
    color: #00bcd4;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    background-color: #2b2b2b;
}

/* ==================== 按钮 ==================== */
QPushButton {
    background-color: #3d3d3d;
    border: 1px solid #555555;
    border-radius: 4px;
    padding: 6px 12px;
    color: #e0e0e0;
    font-weight: bold;
}

QPushButton:hover {
    background-color: #4a4a4a;
    border-color: #00bcd4;
}

QPushButton:pressed {
    background-color: #2d2d2d;
}

QPushButton:disabled {
    background-color: #2b2b2b;
    color: #666666;
    border-color: #3d3d3d;
}

/* 主要按钮（连接、执行等） */
QPushButton#primaryButton {
    background-color: #00796b;
    border-color: #00bcd4;
}

QPushButton#primaryButton:hover {
    background-color: #009688;
}

/* 危险按钮（断开、清除等） */
QPushButton#dangerButton {
    background-color: #c62828;
    border-color: #e53935;
}

QPushButton#dangerButton:hover {
    background-color: #d32f2f;
}

/* ==================== 输入框 ==================== */
QLineEdit, QTextEdit {
    background-color: #1e1e1e;
    border: 1px solid #3d3d3d;
    border-radius: 3px;
    padding: 4px 8px;
    color: #e0e0e0;
    selection-background-color: #00796b;
}

QLineEdit:focus, QTextEdit:focus {
    border-color: #00bcd4;
}

QLineEdit:disabled, QTextEdit:disabled {
    background-color: #2b2b2b;
    color: #666666;
}

/* ==================== 数字输入框 ==================== */
QSpinBox, QDoubleSpinBox {
    background-color: #1e1e1e;
    border: 1px solid #3d3d3d;
    border-radius: 3px;
    padding: 3px 5px;
    color: #e0e0e0;
}

QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #00bcd4;
}

QSpinBox::up-button, QDoubleSpinBox::up-button {
    background-color: #3d3d3d;
    border-left: 1px solid #555555;
    border-bottom: 1px solid #555555;
    border-top-right-radius: 3px;
}

QSpinBox::down-button, QDoubleSpinBox::down-button {
    background-color: #3d3d3d;
    border-left: 1px solid #555555;
    border-bottom-right-radius: 3px;
}

QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
    background-color: #4a4a4a;
}

/* ==================== 下拉框 ==================== */
QComboBox {
    background-color: #1e1e1e;
    border: 1px solid #3d3d3d;
    border-radius: 3px;
    padding: 4px 8px;
    color: #e0e0e0;
}

QComboBox:focus {
    border-color: #00bcd4;
}

QComboBox::drop-down {
    border: none;
    width: 20px;
}

QComboBox::down-arrow {
    image: url(:/icons/arrow_down.png);
    width: 12px;
    height: 12px;
}

QComboBox QAbstractItemView {
    background-color: #2b2b2b;
    border: 1px solid #3d3d3d;
    selection-background-color: #00796b;
    color: #e0e0e0;
}

/* ==================== 滑条 ==================== */
QSlider::groove:horizontal {
    background: #1e1e1e;
    height: 6px;
    border-radius: 3px;
}

QSlider::handle:horizontal {
    background: #00bcd4;
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 8px;
}

QSlider::handle:horizontal:hover {
    background: #00e5ff;
}

QSlider::sub-page:horizontal {
    background: #00796b;
    border-radius: 3px;
}

/* ==================== 标签 ==================== */
QLabel {
    color: #e0e0e0;
    background: transparent;
}

QLabel#statusLabel {
    color: #00bcd4;
    font-weight: bold;
}

QLabel#errorLabel {
    color: #e53935;
    font-weight: bold;
}

QLabel#successLabel {
    color: #4caf50;
    font-weight: bold;
}

/* ==================== 进度条 ==================== */
QProgressBar {
    background-color: #1e1e1e;
    border: 1px solid #3d3d3d;
    border-radius: 3px;
    text-align: center;
    color: #e0e0e0;
}

QProgressBar::chunk {
    background-color: #00796b;
    border-radius: 2px;
}

/* ==================== 表格 ==================== */
QTableWidget {
    background-color: #1e1e1e;
    border: 1px solid #3d3d3d;
    gridline-color: #3d3d3d;
    color: #e0e0e0;
}

QTableWidget::item {
    padding: 4px;
}

QTableWidget::item:selected {
    background-color: #00796b;
}

QHeaderView::section {
    background-color: #2b2b2b;
    border: 1px solid #3d3d3d;
    padding: 4px;
    font-weight: bold;
    color: #00bcd4;
}

/* ==================== 滚动条 ==================== */
QScrollBar:vertical {
    background: #1e1e1e;
    width: 12px;
    border-radius: 6px;
}

QScrollBar::handle:vertical {
    background: #3d3d3d;
    border-radius: 6px;
    min-height: 20px;
}

QScrollBar::handle:vertical:hover {
    background: #4a4a4a;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

QScrollBar:horizontal {
    background: #1e1e1e;
    height: 12px;
    border-radius: 6px;
}

QScrollBar::handle:horizontal {
    background: #3d3d3d;
    border-radius: 6px;
    min-width: 20px;
}

QScrollBar::handle:horizontal:hover {
    background: #4a4a4a;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0px;
}

/* ==================== 选项卡 ==================== */
QTabWidget::pane {
    border: 1px solid #3d3d3d;
    border-radius: 4px;
    background-color: #2b2b2b;
}

QTabBar::tab {
    background-color: #1e1e1e;
    border: 1px solid #3d3d3d;
    padding: 6px 12px;
    margin-right: 2px;
    color: #e0e0e0;
}

QTabBar::tab:selected {
    background-color: #00796b;
    border-bottom-color: #00796b;
}

QTabBar::tab:hover {
    background-color: #3d3d3d;
}

/* ==================== 工具提示 ==================== */
QToolTip {
    background-color: #2b2b2b;
    border: 1px solid #00bcd4;
    color: #e0e0e0;
    padding: 4px;
    border-radius: 3px;
}
"""


def get_qss() -> str:
    """获取样式表"""
    return ROBOT_WINDOW_QSS
