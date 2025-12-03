# scan_qt/gui/robot_window_qss.py

QSS = """
/* 全局字体与背景 */
QWidget {
    background-color: #323232;
    color: #e0e0e0;
    font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
    font-size: 11pt; /* 统一改为 11pt */
}

/* GroupBox 工业风 */
QGroupBox {
    border: 1px solid #505050;
    border-radius: 4px;
    margin-top: 24px; /* 为标题留出空间 */
    padding-top: 10px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    color: #aaa; /* 标题颜色 */
    background-color: #323232; /* 避免文字背景透明 */
    font-weight: bold;
}

/* DockWidget 标题栏优化 */
QDockWidget {
    titlebar-close-icon: url(close.png);
    titlebar-normal-icon: url(float.png);
}
QDockWidget::title {
    text-align: left;
    background: #252525; /* 深黑背景 */
    padding: 8px;
    color: #fff; /* 白色文字 */
    border-bottom: 1px solid #444;
    font-weight: bold;
}

/* 输入框 */
QLineEdit {
    background-color: #404040;
    border: 1px solid #555;
    border-radius: 3px;
    padding: 4px;
    color: #fff;
    selection-background-color: #448aff;
}
QLineEdit:focus { border: 1px solid #448aff; }
QLineEdit:disabled { background-color: #2a2a2a; color: #777; }

/* 按钮通用 */
QPushButton {
    background-color: #454545;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 6px 12px;
}
QPushButton:hover { background-color: #555; border-color: #666; }
QPushButton:pressed { background-color: #222; }

/* 绿色成功按钮 */
QPushButton#btn_success {
    background-color: #2e7d32;
    border-color: #1b5e20;
    color: white;
}
QPushButton#btn_success:hover { background-color: #388e3c; }

/* 蓝色主要按钮 */
QPushButton#btn_primary {
    background-color: #1565c0;
    border-color: #0d47a1;
    color: white;
}
QPushButton#btn_primary:hover { background-color: #1976d2; }

/* 红色危险按钮 */
QPushButton#btn_danger {
    background-color: #c62828;
    border-color: #b71c1c;
    color: white;
}

/* 表格 */
QTableWidget {
    background-color: #2b2b2b;
    gridline-color: #444;
    border: none;
}
QHeaderView::section {
    background-color: #3a3a3a;
    padding: 6px;
    border: 1px solid #505050;
    color: #ccc;
}

/* 分割条 Splitter */
QSplitter::handle {
    background-color: #444;
}
QSplitter::handle:hover {
    background-color: #448aff;
}

/* 滑条 */
QSlider::groove:horizontal {
    border: 1px solid #3a3a3a;
    height: 6px;
    background: #202020;
    margin: 2px 0;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #90caf9;
    border: 1px solid #90caf9;
    width: 14px;
    height: 14px;
    margin: -5px 0; /* handle placed correctly */
    border-radius: 7px;
}
"""
