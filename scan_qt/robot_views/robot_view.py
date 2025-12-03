# scan_qt/robot_views/robot_view.py
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QLabel, QLineEdit, QGroupBox, QDockWidget,
                             QTabWidget, QSlider, QDoubleSpinBox, QTableWidget,
                             QHeaderView, QTextEdit, QCheckBox, QSpinBox, QStatusBar)
from PyQt5.QtCore import Qt
import pyqtgraph as pg


class RobotView(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("就绪")
        self.status_bar.addPermanentWidget(self.status_label)

        # === 中央窗口 (视点表格) ===
        # 工业软件通常将最重要的列表或3D视图放在中间
        self.center_widget = QWidget()
        center_layout = QVBoxLayout(self.center_widget)

        # 视点管理区
        vp_group = QGroupBox("任务管理 / 视点列表 (Task Manager)")
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
        self.btn_ik_move = QPushButton("执行逆解运动 (Execute IK)")
        self.btn_ik_move.setObjectName("btn_success")  # 绿色按钮
        self.btn_ik_move.setMinimumHeight(40)
        action_layout.addWidget(self.btn_ik_move)

        vp_layout.addLayout(input_layout)
        vp_layout.addWidget(self.vp_table)
        vp_layout.addLayout(action_layout)
        vp_group.setLayout(vp_layout)

        center_layout.addWidget(vp_group)
        self.setCentralWidget(self.center_widget)

        # === Dock 1: 连接与控制 (左侧) ===
        self.dock_ctrl = QDockWidget("设备控制 (Device Control)", self)
        self.dock_ctrl.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.dock_ctrl.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)

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

        # 机器人Jog控制
        jog_group = QGroupBox("手动操作 (Jog)")
        jog_layout = QVBoxLayout()
        self.btn_home = QPushButton("回原点 (Home)")

        self.ctrl_tabs = QTabWidget()
        self.tab_fk = QWidget()
        fk_layout = QVBoxLayout(self.tab_fk)

        self.sliders = []
        self.spinboxes = []
        joint_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3", "Table"]

        for name in joint_names:
            row = QHBoxLayout()
            lbl = QLabel(name)
            lbl.setFixedWidth(60)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-36000, 36000)
            spin = QDoubleSpinBox()
            spin.setRange(-360, 360)
            spin.setFixedWidth(70)

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

        # === Dock 2: 数据可视化 (底部) ===
        self.dock_plot = QDockWidget("实时监测 (Monitor)", self)
        self.dock_plot.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        plot_container = QWidget()
        plot_layout = QHBoxLayout(plot_container)

        # 波形图
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('#2b2b2b')

        # 轨迹控制面板
        traj_panel = QWidget()
        traj_panel.setFixedWidth(200)
        traj_vbox = QVBoxLayout(traj_panel)
        traj_group = QGroupBox("轨迹设置")
        traj_g_layout = QVBoxLayout()

        self.chk_show_plot = QCheckBox("显示波形")
        self.chk_show_plot.setChecked(True)
        self.chk_show_traj = QCheckBox("显示3D轨迹")
        self.chk_show_traj.setChecked(True)
        self.btn_traj_color = QPushButton("轨迹颜色")
        self.btn_export_plot = QPushButton("导出3D图(Matlab)")
        self.btn_clear_traj = QPushButton("清除轨迹")

        traj_g_layout.addWidget(self.chk_show_plot)
        traj_g_layout.addWidget(self.chk_show_traj)
        traj_g_layout.addWidget(self.btn_traj_color)
        traj_g_layout.addWidget(self.btn_export_plot)
        traj_g_layout.addWidget(self.btn_clear_traj)
        traj_group.setLayout(traj_g_layout)

        traj_vbox.addWidget(traj_group)
        traj_vbox.addStretch()

        plot_layout.addWidget(traj_panel)
        plot_layout.addWidget(self.plot_widget)

        self.dock_plot.setWidget(plot_container)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.dock_plot)

        # === Dock 3: 日志 (右侧或底部) ===
        self.dock_log = QDockWidget("系统日志", self)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.dock_log.setWidget(self.log_text)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.dock_log)

        # 合并Tab (如果需要)
        self.tabifyDockWidget(self.dock_plot, self.dock_log)
        self.dock_plot.raise_()  # 默认显示波形

    def create_plots(self):
        """初始化波形图"""
        self.plots = []
        self.curves = []
        colors = ['#ff5252', '#69f0ae', '#448aff', '#e040fb', '#ffd740', '#00e5ff', '#ffffff']
        labels = ["J1", "J2", "J3", "J4", "J5", "J6", "Table"]

        for i in range(7):
            p = self.plot_widget.addPlot(row=i // 2, col=i % 2)
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setLabel('left', labels[i])
            curve = p.plot(pen=pg.mkPen(color=colors[i], width=2))
            self.plots.append(p)
            self.curves.append(curve)
