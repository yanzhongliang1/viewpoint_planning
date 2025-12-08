# scan_qt/viewpoints_views/main_window_qss.py
"""
界面样式表
工业白色系设计 (Industrial Light Lab Style)
"""

INDUSTRIAL_LIGHT_QSS = """
/* ==================== 全局样式 ==================== */
QWidget {
    background-color: #f0f2f5; /* 浅灰背景 */
    color: #333333;            /* 深灰文字 */
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
    font-size: 9pt;
}

/* ==================== 分组框 ==================== */
QGroupBox {
    background-color: #ffffff; /* 卡片式白色背景 */
    border: 1px solid #dcdcdc;
    border-radius: 4px;
    margin-top: 10px; /* 留出标题位置 */
    padding-top: 15px;
    font-weight: bold;
    color: #005bb5; /* 工业深蓝 */
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    background-color: transparent; 
}

/* ==================== 按钮 ==================== */
QPushButton {
    background-color: #ffffff;
    border: 1px solid #c0c0c0;
    border-radius: 4px;
    padding: 6px 12px;
    color: #333333;
    font-weight: normal;
}

QPushButton:hover {
    background-color: #e6f7ff; /* 浅蓝悬停 */
    border-color: #1890ff;
    color: #1890ff;
}

QPushButton:pressed {
    background-color: #d6e4ff;
}

QPushButton:disabled {
    background-color: #f5f5f5;
    color: #bbbbbb;
    border-color: #e0e0e0;
}

/* 主要按钮（连接、执行等） - 强调色 */
QPushButton#primaryButton {
    background-color: #1890ff; /* 亮蓝 */
    border-color: #1890ff;
    color: #ffffff;
    font-weight: bold;
}

QPushButton#primaryButton:hover {
    background-color: #40a9ff;
    border-color: #40a9ff;
}

QPushButton#primaryButton:pressed {
    background-color: #096dd9;
}

/* 危险按钮（删除等） */
QPushButton#dangerButton {
    background-color: #ffffff;
    border-color: #ff4d4f;
    color: #ff4d4f;
}

QPushButton#dangerButton:hover {
    background-color: #fff1f0;
    color: #ff7875;
}

QSplitter::handle {
    background-color: #dcdcdc;
    height: 1px;
}

/* ==================== 输入框 ==================== */
QLineEdit, QTextEdit {
    background-color: #ffffff;
    border: 1px solid #d9d9d9;
    border-radius: 3px;
    padding: 4px 8px;
    color: #333333;
    selection-background-color: #1890ff;
    selection-color: #ffffff;
}

QLineEdit:focus, QTextEdit:focus {
    border-color: #40a9ff;
}

QLineEdit:disabled, QTextEdit:disabled {
    background-color: #f5f5f5;
    color: #bbbbbb;
}

/* ==================== 数字输入框 ==================== */
QSpinBox, QDoubleSpinBox {
    background-color: #ffffff;
    border: 1px solid #d9d9d9;
    border-radius: 3px;
    padding: 3px 5px;
    color: #333333;
}

QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #40a9ff;
}

QSpinBox::up-button, QDoubleSpinBox::up-button {
    background-color: #f0f0f0;
    border-left: 1px solid #d9d9d9;
    border-bottom: 1px solid #d9d9d9;
    border-top-right-radius: 3px;
    image: none; /* 使用默认或自定义图片，这里简化 */
}

QSpinBox::down-button, QDoubleSpinBox::down-button {
    background-color: #f0f0f0;
    border-left: 1px solid #d9d9d9;
    border-bottom-right-radius: 3px;
}

QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
    background-color: #e6f7ff;
}

/* ==================== 下拉框 ==================== */
QComboBox {
    background-color: #ffffff;
    border: 1px solid #d9d9d9;
    border-radius: 3px;
    padding: 4px 8px;
    color: #333333;
}

QComboBox:focus {
    border-color: #40a9ff;
}

QComboBox::drop-down {
    border: none;
    width: 20px;
    background: transparent;
}

QComboBox QAbstractItemView {
    background-color: #ffffff;
    border: 1px solid #d9d9d9;
    selection-background-color: #e6f7ff;
    selection-color: #333333;
    color: #333333;
    outline: 0px;
}

/* ==================== 标签 ==================== */
QLabel {
    color: #333333;
    background: transparent;
}

QLabel#statusLabel { color: #1890ff; font-weight: bold; }
QLabel#errorLabel { color: #ff4d4f; font-weight: bold; }
QLabel#successLabel { color: #52c41a; font-weight: bold; }

/* ==================== 表格 ==================== */
QTableWidget {
    background-color: #ffffff;
    border: 1px solid #dcdcdc;
    gridline-color: #f0f0f0;
    color: #333333;
    alternate-background-color: #fafafa;
}

QTableWidget::item {
    padding: 4px;
    border-bottom: 1px solid #f0f0f0;
}

QTableWidget::item:selected {
    background-color: #e6f7ff; /* 选中时浅蓝背景 */
    color: #000000;
}

QHeaderView::section {
    background-color: #f7f7f7;
    border: none;
    border-bottom: 1px solid #dcdcdc;
    border-right: 1px solid #e8e8e8;
    padding: 6px 4px;
    font-weight: bold;
    color: #666666;
}

/* ==================== 滚动条 ==================== */
QScrollBar:vertical {
    background: #f1f1f1;
    width: 10px;
}

QScrollBar::handle:vertical {
    background: #c1c1c1;
    border-radius: 5px;
}

QScrollBar::handle:vertical:hover {
    background: #a8a8a8;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* ==================== 选项卡 ==================== */
QTabWidget::pane {
    border: 1px solid #dcdcdc;
    border-top: 2px solid #1890ff; /* 顶部亮条 */
    background-color: #ffffff;
}

QTabBar::tab {
    background-color: #f0f2f5;
    border: 1px solid #dcdcdc;
    border-bottom: none;
    padding: 8px 16px;
    margin-right: 2px;
    color: #666666;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
}

QTabBar::tab:selected {
    background-color: #ffffff;
    color: #1890ff;
    font-weight: bold;
    border-bottom: 1px solid #ffffff; /* 遮住下边框实现连通 */
}

QTabBar::tab:hover:!selected {
    background-color: #e6f7ff;
    color: #1890ff;
}

/* ==================== 工具提示 ==================== */
QToolTip {
    background-color: #ffffff;
    border: 1px solid #c0c0c0;
    color: #333333;
    padding: 4px;
}
"""


def get_qss() -> str:
    """获取样式表"""
    return INDUSTRIAL_LIGHT_QSS
