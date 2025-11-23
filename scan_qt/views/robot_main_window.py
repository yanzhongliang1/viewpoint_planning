# scan_qt/views/robot_main_window.py
"""
机器人控制主界面
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QLineEdit, QDoubleSpinBox,
    QComboBox, QGridLayout, QSlider, QFileDialog,
    QTextEdit, QProgressBar, QCheckBox, QTableWidget,
    QTableWidgetItem, QHeaderView, QSplitter
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont

from scan_qt.robot.robot_comm import RobotComm
from scan_qt.robot.robot_model import RobotModel
from scan_qt.robot.robot_ik import RobotIK
from scan_qt.robot.robot_worker import RobotWorker
from scan_qt.robot.viewpoints_parser import ViewpointParser
from scan_qt.views.robot_main_window_qss import get_qss


class RobotMainWindow(QWidget):
    """
    机器人控制主界面

    功能模块:
    1. 连接控制
    2. 视点管理
    3. 坐标系显示与转换
    4. 关节 JOG 控制
    5. IK 求解与执行
    6. 轨迹可视化
    """

    # 自定义信号
    connection_changed = pyqtSignal(bool)  # 连接状态变化

    def __init__(self, parent=None):
        super().__init__(parent)

        # 核心对象
        self.comm: RobotComm = None
        self.model: RobotModel = None
        self.ik_solver: RobotIK = None
        self.worker: RobotWorker = None

        # 视点数据
        self.viewpoints = []  # [(name, position, direction), ...]
        self.ik_results = []  # [(name, success, ur5_config, turtle_config), ...]

        # UI 组件引用
        self.ui_components = {}

        # 初始化界面
        self._init_ui()
        self._apply_styles()

        # 延迟导入槽函数（避免循环导入）
        from scan_qt.views.robot_main_window_slots import RobotMainWindowSlots
        self.slots = RobotMainWindowSlots(self)
        self._connect_signals()

    # ==================== UI 初始化 ====================

    def _init_ui(self):
        """初始化界面"""
        self.setWindowTitle("机器人控制系统")
        self.setMinimumSize(1400, 900)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # 1. 顶部：连接控制
        main_layout.addWidget(self._create_connection_group())

        # 2. 中部：主要内容区（使用分割器）
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._create_left_panel())
        splitter.addWidget(self._create_center_panel())
        splitter.addWidget(self._create_right_panel())
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setStretchFactor(2, 2)

        main_layout.addWidget(splitter, stretch=1)

        # 3. 底部：状态栏
        main_layout.addWidget(self._create_status_bar())

    def _create_connection_group(self) -> QGroupBox:
        """创建连接控制组"""
        group = QGroupBox("CoppeliaSim 连接")
        layout = QHBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        # Host
        layout.addWidget(QLabel("Host:"))
        self.ui_components['edit_host'] = QLineEdit("localhost")
        self.ui_components['edit_host'].setMaximumWidth(150)
        layout.addWidget(self.ui_components['edit_host'])

        # Port
        layout.addWidget(QLabel("Port:"))
        self.ui_components['edit_port'] = QLineEdit("23000")
        self.ui_components['edit_port'].setMaximumWidth(80)
        layout.addWidget(self.ui_components['edit_port'])

        layout.addSpacing(20)

        # 连接按钮
        self.ui_components['btn_connect'] = QPushButton("连接")
        self.ui_components['btn_connect'].setObjectName("primaryButton")
        self.ui_components['btn_connect'].setMaximumWidth(100)
        layout.addWidget(self.ui_components['btn_connect'])

        # 断开按钮
        self.ui_components['btn_disconnect'] = QPushButton("断开")
        self.ui_components['btn_disconnect'].setObjectName("dangerButton")
        self.ui_components['btn_disconnect'].setMaximumWidth(100)
        self.ui_components['btn_disconnect'].setEnabled(False)
        layout.addWidget(self.ui_components['btn_disconnect'])

        layout.addStretch(1)

        # 状态标签
        self.ui_components['label_connection_status'] = QLabel("未连接")
        self.ui_components['label_connection_status'].setObjectName("statusLabel")
        layout.addWidget(self.ui_components['label_connection_status'])

        return group

    def _create_left_panel(self) -> QWidget:
        """创建左侧面板（视点管理 + 坐标系）"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # 视点管理
        layout.addWidget(self._create_viewpoint_group())

        # 坐标系信息
        layout.addWidget(self._create_frame_group())

        layout.addStretch(1)

        return widget

    def _create_viewpoint_group(self) -> QGroupBox:
        """创建视点管理组"""
        group = QGroupBox("视点管理")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 文件导入
        file_layout = QHBoxLayout()
        self.ui_components['btn_load_viewpoints'] = QPushButton("导入视点文件")
        self.ui_components['btn_load_viewpoints'].setMaximumWidth(120)
        file_layout.addWidget(self.ui_components['btn_load_viewpoints'])

        self.ui_components['label_viewpoint_file'] = QLabel("未加载")
        file_layout.addWidget(self.ui_components['label_viewpoint_file'], stretch=1)
        layout.addLayout(file_layout)

        # 视点列表
        self.ui_components['table_viewpoints'] = QTableWidget()
        self.ui_components['table_viewpoints'].setColumnCount(4)
        self.ui_components['table_viewpoints'].setHorizontalHeaderLabels(
            ["名称", "位置 (m)", "方向", "状态"]
        )
        self.ui_components['table_viewpoints'].horizontalHeader().setStretchLastSection(True)
        self.ui_components['table_viewpoints'].setMaximumHeight(200)
        self.ui_components['table_viewpoints'].setSelectionBehavior(QTableWidget.SelectRows)
        layout.addWidget(self.ui_components['table_viewpoints'])

        # 手动输入
        manual_layout = QGridLayout()
        manual_layout.addWidget(QLabel("手动输入:"), 0, 0)

        self.ui_components['edit_manual_viewpoint'] = QLineEdit()
        self.ui_components['edit_manual_viewpoint'].setPlaceholderText(
            "格式: name x y z dx dy dz (mm)"
        )
        manual_layout.addWidget(self.ui_components['edit_manual_viewpoint'], 0, 1, 1, 2)

        self.ui_components['btn_add_manual_viewpoint'] = QPushButton("添加")
        self.ui_components['btn_add_manual_viewpoint'].setMaximumWidth(80)
        manual_layout.addWidget(self.ui_components['btn_add_manual_viewpoint'], 0, 3)

        layout.addLayout(manual_layout)

        # Dummy 控制
        dummy_layout = QHBoxLayout()
        self.ui_components['btn_create_dummies'] = QPushButton("创建 Dummy")
        self.ui_components['btn_create_dummies'].setMaximumWidth(120)
        dummy_layout.addWidget(self.ui_components['btn_create_dummies'])

        self.ui_components['btn_clear_dummies'] = QPushButton("清除 Dummy")
        self.ui_components['btn_clear_dummies'].setObjectName("dangerButton")
        self.ui_components['btn_clear_dummies'].setMaximumWidth(120)
        dummy_layout.addWidget(self.ui_components['btn_clear_dummies'])

        dummy_layout.addStretch(1)
        layout.addLayout(dummy_layout)

        return group

    def _create_frame_group(self) -> QGroupBox:
        """创建坐标系信息组"""
        group = QGroupBox("坐标系位姿")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 坐标系表格
        self.ui_components['table_frames'] = QTableWidget()
        self.ui_components['table_frames'].setColumnCount(2)
        self.ui_components['table_frames'].setHorizontalHeaderLabels(["坐标系", "位姿"])
        self.ui_components['table_frames'].horizontalHeader().setStretchLastSection(True)
        self.ui_components['table_frames'].setMaximumHeight(180)

        # 添加行
        frames = [
            ("W - 世界", ""),
            ("B - 基座", ""),
            ("J - 转台", ""),
            ("O - 工件", ""),
            ("S - 扫描仪", "")
        ]

        self.ui_components['table_frames'].setRowCount(len(frames))
        for i, (name, _) in enumerate(frames):
            self.ui_components['table_frames'].setItem(i, 0, QTableWidgetItem(name))
            self.ui_components['table_frames'].setItem(i, 1, QTableWidgetItem(""))

        layout.addWidget(self.ui_components['table_frames'])

        # 刷新按钮
        self.ui_components['btn_refresh_frames'] = QPushButton("刷新坐标系")
        self.ui_components['btn_refresh_frames'].setMaximumWidth(120)
        layout.addWidget(self.ui_components['btn_refresh_frames'])

        return group

    def _create_center_panel(self) -> QWidget:
        """创建中间面板（关节控制 + IK）"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # 关节 JOG 控制
        layout.addWidget(self._create_joint_jog_group())

        # IK 控制
        layout.addWidget(self._create_ik_group())

        return widget

    def _create_joint_jog_group(self) -> QGroupBox:
        """创建关节 JOG 控制组"""
        group = QGroupBox("关节 JOG 控制")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 关节滑条
        joint_names = [f"joint{i}" for i in range(1, 7)] + ["turtle_joint"]

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        self.ui_components['sliders_joints'] = {}
        self.ui_components['labels_joint_values'] = {}

        for i, name in enumerate(joint_names):
            # 标签
            label = QLabel(f"{name}:")
            label.setMinimumWidth(80)
            grid.addWidget(label, i, 0)

            # 滑条
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.setTickPosition(QSlider.NoTicks)
            self.ui_components['sliders_joints'][name] = slider
            grid.addWidget(slider, i, 1)

            # 数值显示
            value_label = QLabel("0.0°")
            value_label.setMinimumWidth(60)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.ui_components['labels_joint_values'][name] = value_label
            grid.addWidget(value_label, i, 2)

        layout.addLayout(grid)

        # 控制按钮
        btn_layout = QHBoxLayout()

        self.ui_components['btn_refresh_joints'] = QPushButton("刷新关节")
        self.ui_components['btn_refresh_joints'].setMaximumWidth(100)
        btn_layout.addWidget(self.ui_components['btn_refresh_joints'])

        self.ui_components['btn_go_home'] = QPushButton("回 Home")
        self.ui_components['btn_go_home'].setMaximumWidth(100)
        btn_layout.addWidget(self.ui_components['btn_go_home'])

        btn_layout.addStretch(1)
        layout.addLayout(btn_layout)

        return group

    def _create_ik_group(self) -> QGroupBox:
        """创建 IK 控制组"""
        group = QGroupBox("逆运动学求解")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 参数设置
        param_layout = QGridLayout()

        param_layout.addWidget(QLabel("接近距离 (mm):"), 0, 0)
        self.ui_components['spin_approach_distance'] = QDoubleSpinBox()
        self.ui_components['spin_approach_distance'].setRange(0, 500)
        self.ui_components['spin_approach_distance'].setValue(50)
        self.ui_components['spin_approach_distance'].setSingleStep(10)
        self.ui_components['spin_approach_distance'].setMaximumWidth(100)
        param_layout.addWidget(self.ui_components['spin_approach_distance'], 0, 1)

        param_layout.addWidget(QLabel("运动时长 (s):"), 0, 2)
        self.ui_components['spin_motion_duration'] = QDoubleSpinBox()
        self.ui_components['spin_motion_duration'].setRange(0.5, 10.0)
        self.ui_components['spin_motion_duration'].setValue(2.0)
        self.ui_components['spin_motion_duration'].setSingleStep(0.5)
        self.ui_components['spin_motion_duration'].setMaximumWidth(100)
        param_layout.addWidget(self.ui_components['spin_motion_duration'], 0, 3)

        layout.addLayout(param_layout)

        # IK 求解按钮
        ik_btn_layout = QHBoxLayout()

        self.ui_components['btn_solve_ik_selected'] = QPushButton("求解选中视点")
        self.ui_components['btn_solve_ik_selected'].setMaximumWidth(130)
        ik_btn_layout.addWidget(self.ui_components['btn_solve_ik_selected'])

        self.ui_components['btn_solve_ik_all'] = QPushButton("批量求解全部")
        self.ui_components['btn_solve_ik_all'].setObjectName("primaryButton")
        self.ui_components['btn_solve_ik_all'].setMaximumWidth(130)
        ik_btn_layout.addWidget(self.ui_components['btn_solve_ik_all'])

        ik_btn_layout.addStretch(1)
        layout.addLayout(ik_btn_layout)

        # 执行按钮
        exec_btn_layout = QHBoxLayout()

        self.ui_components['btn_move_to_selected'] = QPushButton("运动到选中")
        self.ui_components['btn_move_to_selected'].setMaximumWidth(130)
        exec_btn_layout.addWidget(self.ui_components['btn_move_to_selected'])

        self.ui_components['btn_execute_trajectory'] = QPushButton("执行完整轨迹")
        self.ui_components['btn_execute_trajectory'].setObjectName("primaryButton")
        self.ui_components['btn_execute_trajectory'].setMaximumWidth(130)
        exec_btn_layout.addWidget(self.ui_components['btn_execute_trajectory'])

        exec_btn_layout.addStretch(1)
        layout.addLayout(exec_btn_layout)

        # IK 结果表格
        self.ui_components['table_ik_results'] = QTableWidget()
        self.ui_components['table_ik_results'].setColumnCount(3)
        self.ui_components['table_ik_results'].setHorizontalHeaderLabels(
            ["视点", "状态", "配置预览"]
        )
        self.ui_components['table_ik_results'].horizontalHeader().setStretchLastSection(True)
        self.ui_components['table_ik_results'].setMaximumHeight(150)
        layout.addWidget(self.ui_components['table_ik_results'])

        return group

    def _create_right_panel(self) -> QWidget:
        """创建右侧面板（轨迹可视化 + 日志）"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # 轨迹可视化
        layout.addWidget(self._create_trajectory_group())

        # 日志输出
        layout.addWidget(self._create_log_group())

        return widget

    def _create_trajectory_group(self) -> QGroupBox:
        """创建轨迹可视化组"""
        group = QGroupBox("轨迹可视化")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 显示控制
        control_layout = QHBoxLayout()

        self.ui_components['check_show_trajectory'] = QCheckBox("显示轨迹线")
        self.ui_components['check_show_trajectory'].setChecked(True)
        control_layout.addWidget(self.ui_components['check_show_trajectory'])

        self.ui_components['btn_update_trajectory'] = QPushButton("更新轨迹")
        self.ui_components['btn_update_trajectory'].setMaximumWidth(100)
        control_layout.addWidget(self.ui_components['btn_update_trajectory'])

        self.ui_components['btn_clear_trajectory'] = QPushButton("清除轨迹")
        self.ui_components['btn_clear_trajectory'].setObjectName("dangerButton")
        self.ui_components['btn_clear_trajectory'].setMaximumWidth(100)
        control_layout.addWidget(self.ui_components['btn_clear_trajectory'])

        control_layout.addStretch(1)
        layout.addLayout(control_layout)

        # 轨迹信息
        info_layout = QGridLayout()

        info_layout.addWidget(QLabel("视点数量:"), 0, 0)
        self.ui_components['label_trajectory_count'] = QLabel("0")
        info_layout.addWidget(self.ui_components['label_trajectory_count'], 0, 1)

        info_layout.addWidget(QLabel("成功求解:"), 1, 0)
        self.ui_components['label_trajectory_success'] = QLabel("0")
        self.ui_components['label_trajectory_success'].setObjectName("successLabel")
        info_layout.addWidget(self.ui_components['label_trajectory_success'], 1, 1)

        info_layout.addWidget(QLabel("总路径长度:"), 2, 0)
        self.ui_components['label_trajectory_length'] = QLabel("0.0 m")
        info_layout.addWidget(self.ui_components['label_trajectory_length'], 2, 1)

        layout.addLayout(info_layout)

        return group

    def _create_log_group(self) -> QGroupBox:
        """创建日志输出组"""
        group = QGroupBox("系统日志")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 日志文本框
        self.ui_components['text_log'] = QTextEdit()
        self.ui_components['text_log'].setReadOnly(True)
        self.ui_components['text_log'].setMaximumHeight(300)

        # 设置等宽字体
        font = QFont("Consolas", 9)
        self.ui_components['text_log'].setFont(font)

        layout.addWidget(self.ui_components['text_log'])

        # 清除按钮
        self.ui_components['btn_clear_log'] = QPushButton("清除日志")
        self.ui_components['btn_clear_log'].setMaximumWidth(100)
        layout.addWidget(self.ui_components['btn_clear_log'])

        return group

    def _create_status_bar(self) -> QWidget:
        """创建状态栏"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        # 状态标签
        self.ui_components['label_status'] = QLabel("就绪")
        self.ui_components['label_status'].setObjectName("statusLabel")
        layout.addWidget(self.ui_components['label_status'])

        layout.addStretch(1)

        # 进度条
        self.ui_components['progress_bar'] = QProgressBar()
        self.ui_components['progress_bar'].setMaximumWidth(300)
        self.ui_components['progress_bar'].setMaximumHeight(20)
        self.ui_components['progress_bar'].setValue(0)
        self.ui_components['progress_bar'].setVisible(False)
        layout.addWidget(self.ui_components['progress_bar'])

        return widget

    # ==================== 样式应用 ====================

    def _apply_styles(self):
        """应用样式表"""
        self.setStyleSheet(get_qss())

    # ==================== 信号连接 ====================

    def _connect_signals(self):
        """连接信号与槽"""
        # 连接控制
        self.ui_components['btn_connect'].clicked.connect(self.slots.on_connect_clicked)
        self.ui_components['btn_disconnect'].clicked.connect(self.slots.on_disconnect_clicked)

        # 视点管理
        self.ui_components['btn_load_viewpoints'].clicked.connect(self.slots.on_load_viewpoints_clicked)
        self.ui_components['btn_add_manual_viewpoint'].clicked.connect(self.slots.on_add_manual_viewpoint_clicked)
        self.ui_components['btn_create_dummies'].clicked.connect(self.slots.on_create_dummies_clicked)
        self.ui_components['btn_clear_dummies'].clicked.connect(self.slots.on_clear_dummies_clicked)

        # 坐标系
        self.ui_components['btn_refresh_frames'].clicked.connect(self.slots.on_refresh_frames_clicked)

        # 关节控制
        self.ui_components['btn_refresh_joints'].clicked.connect(self.slots.on_refresh_joints_clicked)
        self.ui_components['btn_go_home'].clicked.connect(self.slots.on_go_home_clicked)

        # 关节滑条
        for name, slider in self.ui_components['sliders_joints'].items():
            slider.valueChanged.connect(
                lambda val, n=name: self.slots.on_joint_slider_changed(n, val)
            )

        # IK 控制
        self.ui_components['btn_solve_ik_selected'].clicked.connect(self.slots.on_solve_ik_selected_clicked)
        self.ui_components['btn_solve_ik_all'].clicked.connect(self.slots.on_solve_ik_all_clicked)
        self.ui_components['btn_move_to_selected'].clicked.connect(self.slots.on_move_to_selected_clicked)
        self.ui_components['btn_execute_trajectory'].clicked.connect(self.slots.on_execute_trajectory_clicked)

        # 轨迹可视化
        self.ui_components['check_show_trajectory'].stateChanged.connect(self.slots.on_trajectory_visibility_changed)
        self.ui_components['btn_update_trajectory'].clicked.connect(self.slots.on_update_trajectory_clicked)
        self.ui_components['btn_clear_trajectory'].clicked.connect(self.slots.on_clear_trajectory_clicked)

        # 日志
        self.ui_components['btn_clear_log'].clicked.connect(self.slots.on_clear_log_clicked)

    # ==================== 公共方法 ====================

    def log_message(self, message: str, level: str = "INFO"):
        """
        添加日志消息

        Args:
            message: 消息内容
            level: 日志级别 (INFO/WARNING/ERROR/SUCCESS)
        """
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")

        color_map = {
            "INFO": "#e0e0e0",
            "WARNING": "#ffa726",
            "ERROR": "#e53935",
            "SUCCESS": "#4caf50"
        }

        color = color_map.get(level, "#e0e0e0")
        formatted_message = f'<span style="color: {color};">[{timestamp}] [{level}] {message}</span>'

        self.ui_components['text_log'].append(formatted_message)

        # 自动滚动到底部
        scrollbar = self.ui_components['text_log'].verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def set_status(self, message: str, show_progress: bool = False):
        """设置状态栏消息"""
        self.ui_components['label_status'].setText(message)
        self.ui_components['progress_bar'].setVisible(show_progress)

    def update_progress(self, value: int):
        """更新进度条"""
        self.ui_components['progress_bar'].setValue(value)

    def closeEvent(self, event):
        """窗口关闭事件"""
        if self.worker is not None and self.worker.isRunning():
            self.worker.stop()

        if self.comm is not None and self.comm.is_connected():
            self.comm.disconnect()

        event.accept()
