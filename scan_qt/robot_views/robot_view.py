# scan_qt/robot_views/robot_view.py
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QLabel, QLineEdit, QGroupBox, QDockWidget,
                             QTabWidget, QSlider, QDoubleSpinBox, QTableWidget,
                             QHeaderView, QCheckBox, QStatusBar)
from PyQt5.QtCore import Qt
import pyqtgraph as pg

# 导入您的日志组件
from scan_qt.robot_views.logger import LogWidget


class RobotView(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        # === 关键：布局优先级设置，防止底部 Dock 被侧边遮挡 ===
        self.setCorner(Qt.BottomLeftCorner, Qt.BottomDockWidgetArea)
        self.setCorner(Qt.BottomRightCorner, Qt.BottomDockWidgetArea)

        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("就绪 / Ready")
        self.status_bar.addPermanentWidget(self.status_label)

        # === 中央窗口 (任务管理) ===
        self.center_widget = QWidget()
        center_layout = QVBoxLayout(self.center_widget)

        vp_group = QGroupBox("任务管理 / 视点列表")
        vp_layout = QVBoxLayout()

        input_layout = QHBoxLayout()
        self.input_vp_pos = QLineEdit("0.5, 0, 0.5")
        self.input_vp_pos.setPlaceholderText("x, y, z")
        self.btn_create_manual = QPushButton("手动添加点")
        self.btn_load_file = QPushButton("加载文件...")
        input_layout.addWidget(QLabel("坐标:"))
        input_layout.addWidget(self.input_vp_pos)
        input_layout.addWidget(self.btn_create_manual)
        input_layout.addWidget(self.btn_load_file)

        self.vp_table = QTableWidget()
        self.vp_table.setColumnCount(3)
        self.vp_table.setHorizontalHeaderLabels(["ID (Handle)", "Name", "Position (World)"])
        self.vp_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.vp_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.vp_table.setAlternatingRowColors(True)

        action_layout = QHBoxLayout()
        # 按钮名称体现 AutoScanner 逻辑
        self.btn_ik_move = QPushButton("单步逆解")
        self.btn_ik_move.setObjectName("btn_success")
        self.btn_ik_move.setMinimumHeight(40)

        self.btn_full_scan = QPushButton("全自动扫描")
        self.btn_full_scan.setObjectName("btn_primary")  # 假设您有对应的QSS，或者留空
        self.btn_full_scan.setMinimumHeight(40)

        self.btn_stop_scan = QPushButton("停止 (Stop)")
        self.btn_stop_scan.setObjectName("btn_danger")
        self.btn_stop_scan.setMinimumHeight(40)
        self.btn_stop_scan.setEnabled(False)  # 初始不可用

        action_layout.addWidget(self.btn_ik_move)
        action_layout.addWidget(self.btn_full_scan)
        action_layout.addWidget(self.btn_stop_scan)

        vp_layout.addLayout(input_layout)
        vp_layout.addWidget(self.vp_table)
        vp_layout.addLayout(action_layout)
        vp_group.setLayout(vp_layout)

        center_layout.addWidget(vp_group)
        self.setCentralWidget(self.center_widget)

        # === Dock 1: 设备控制 (左侧) ===
        self.dock_ctrl = QDockWidget("设备控制", self)
        self.dock_ctrl.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)

        ctrl_widget = QWidget()
        ctrl_layout = QVBoxLayout(ctrl_widget)

        # 连接模块
        conn_group = QGroupBox("通讯连接")
        conn_layout = QGridLayout()
        self.edit_host = QLineEdit("localhost")
        self.edit_port = QLineEdit("23000")
        self.btn_connect = QPushButton("连接系统")
        self.btn_connect.setObjectName("btn_primary")
        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.setObjectName("btn_danger")
        self.btn_disconnect.setEnabled(False)

        conn_layout.addWidget(QLabel("IP:"), 0, 0)
        conn_layout.addWidget(self.edit_host, 0, 1)
        conn_layout.addWidget(QLabel("Port:"), 0, 2)
        conn_layout.addWidget(self.edit_port, 0, 3)
        conn_layout.addWidget(self.btn_connect, 1, 0, 1, 2)
        conn_layout.addWidget(self.btn_disconnect, 1, 2, 1, 2)
        conn_group.setLayout(conn_layout)

        # Jog控制
        jog_group = QGroupBox("手动操作 (Jog)")
        jog_layout = QVBoxLayout()
        self.btn_home = QPushButton("回原点 (Home)")

        self.ctrl_tabs = QTabWidget()
        self.tab_fk = QWidget()
        fk_layout = QVBoxLayout(self.tab_fk)
        fk_layout.setContentsMargins(2, 2, 2, 2)

        self.sliders = []
        self.spinboxes = []
        joint_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3", "Table"]

        for name in joint_names:
            row = QHBoxLayout()
            lbl = QLabel(name)
            lbl.setFixedWidth(55)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-36000, 36000)
            spin = QDoubleSpinBox()
            spin.setRange(-360, 360)
            spin.setFixedWidth(65)
            spin.setDecimals(2)

            self.sliders.append(slider)
            self.spinboxes.append(spin)

            row.addWidget(lbl)
            row.addWidget(slider)
            row.addWidget(spin)
            fk_layout.addLayout(row)

        self.ctrl_tabs.addTab(self.tab_fk, "关节调试")
        jog_layout.addWidget(self.btn_home)
        jog_layout.addWidget(self.ctrl_tabs)
        jog_group.setLayout(jog_layout)

        ctrl_layout.addWidget(conn_group)
        ctrl_layout.addWidget(jog_group)
        ctrl_layout.addStretch()

        self.dock_ctrl.setWidget(ctrl_widget)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.dock_ctrl)

        # === Dock 2: 监测与日志 (底部) ===
        self.dock_plot = QDockWidget("实时监测", self)
        self.dock_plot.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        plot_container = QWidget()
        plot_layout = QHBoxLayout(plot_container)
        plot_layout.setContentsMargins(0, 0, 0, 0)

        # 轨迹设置面板
        traj_panel = QWidget()
        traj_panel.setFixedWidth(150)
        traj_vbox = QVBoxLayout(traj_panel)
        traj_group = QGroupBox("显示设置")
        traj_g_layout = QVBoxLayout()

        self.chk_show_plot = QCheckBox("显示波形")
        self.chk_show_plot.setChecked(True)
        self.chk_show_traj = QCheckBox("显示3D轨迹")
        self.chk_show_traj.setChecked(True)
        self.btn_export_plot = QPushButton("导出图表")
        self.btn_clear_traj = QPushButton("清除轨迹")

        traj_g_layout.addWidget(self.chk_show_plot)
        traj_g_layout.addWidget(self.chk_show_traj)
        traj_g_layout.addWidget(self.btn_export_plot)
        traj_g_layout.addWidget(self.btn_clear_traj)
        traj_group.setLayout(traj_g_layout)

        traj_vbox.addWidget(traj_group)
        traj_vbox.addStretch()

        # 波形组件
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('#222')

        plot_layout.addWidget(traj_panel)
        plot_layout.addWidget(self.plot_widget)

        self.dock_plot.setWidget(plot_container)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.dock_plot)

        # 嵌入您的日志组件
        self.dock_log = QDockWidget("系统日志", self)
        self.log_widget = LogWidget()  # 使用您的类
        self.dock_log.setWidget(self.log_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.dock_log)

        # 并排显示
        self.splitDockWidget(self.dock_plot, self.dock_log, Qt.Horizontal)

    def create_plots(self):
        """初始化波形图: 7行1列布局"""
        self.plots = []
        self.curves = []
        colors = ['#ff5252', '#69f0ae', '#448aff', '#e040fb', '#ffd740', '#00e5ff', '#ffffff']
        labels = ["Base", "Shld", "Elbow", "W1", "W2", "W3", "Table"]

        self.plot_widget.clear()
        for i in range(7):
            p = self.plot_widget.addPlot(row=i, col=0)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel('left', labels[i], units='°')
            if i < 6: p.hideAxis('bottom')
            p.setMouseEnabled(x=False, y=False)
            p.setMenuEnabled(False)

            curve = p.plot(pen=pg.mkPen(color=colors[i], width=2))
            self.plots.append(p)
            self.curves.append(curve)
