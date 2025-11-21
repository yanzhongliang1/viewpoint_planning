# scan_qt/views/robot_main_window.py
import numpy as np

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QLineEdit, QDoubleSpinBox,
    QComboBox, QGridLayout, QSpinBox, QSlider
)
from PyQt5.QtCore import Qt

from scan_qt.robot.robot_comm import RobotComm
from scan_qt.robot.robot_model import RobotModel
from scan_qt.robot.robot_planner import RobotPlanner
from scan_qt.views.robot_main_window_slots import RobotMainWindowSlots


class RobotMainWidget(QWidget):
    """
    机器人通讯 + 坐标系 + 七轴规划页面：
      - 连接/断开 CoppeliaSim
      - 显示 {W,J,O,S} 位姿 & 坐标变换测试
      - 显示 UR5+转台关节状态 & 关节空间规划 + JOG 拖动
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # 方便在全局 QSS 中单独定制机器人页面
        self.setObjectName("RobotPage")

        self.comm: RobotComm | None = None
        self.robot_model: RobotModel | None = None
        self.robot_planner: RobotPlanner | None = None

        self.slots = RobotMainWindowSlots(self)

        self._init_ui()
        self._connect_slots()

    # ----------------- UI 搭建 -----------------

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        # =========================================================
        # 1. 顶部：连接控制
        # =========================================================
        conn_group = QGroupBox("CoppeliaSim 连接")
        conn_layout = QHBoxLayout(conn_group)
        conn_layout.setContentsMargins(6, 4, 6, 4)
        conn_layout.setSpacing(6)

        conn_layout.addWidget(QLabel("Host:"))
        self.edit_host = QLineEdit("127.0.0.1")
        self.edit_host.setMaximumWidth(120)
        conn_layout.addWidget(self.edit_host)

        conn_layout.addWidget(QLabel("Port:"))
        self.edit_port = QLineEdit("19997")
        self.edit_port.setMaximumWidth(80)
        conn_layout.addWidget(self.edit_port)

        conn_layout.addSpacing(8)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setMaximumWidth(80)
        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.setMaximumWidth(80)
        self.btn_disconnect.setEnabled(False)

        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_disconnect)

        conn_layout.addStretch(1)

        self.label_status = QLabel("未连接")
        conn_layout.addWidget(self.label_status)

        main_layout.addWidget(conn_group)

        # =========================================================
        # 2. 中部：左右布局
        #    左：坐标系 & 变换测试
        #    右：关节状态 & 规划 & JOG
        # =========================================================
        center_layout = QHBoxLayout()
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(8)
        main_layout.addLayout(center_layout, stretch=1)

        left_layout = QVBoxLayout()
        left_layout.setSpacing(8)
        center_layout.addLayout(left_layout, stretch=1)

        right_layout = QVBoxLayout()
        right_layout.setSpacing(8)
        center_layout.addLayout(right_layout, stretch=1)

        # ---------------- 左列：坐标系信息 ----------------
        frame_group = QGroupBox("坐标系位姿（相对于世界系 {W}）")
        frame_layout = QGridLayout(frame_group)
        frame_layout.setHorizontalSpacing(6)
        frame_layout.setVerticalSpacing(3)
        frame_layout.setContentsMargins(8, 8, 8, 8)

        self.labels_frame = {}
        frames = ["W", "J", "O", "S", "B"]
        for i, name in enumerate(frames):
            base_row = i * 2

            # 第一行：名称 + pos
            lbl_name = QLabel(f"{name}:")
            frame_layout.addWidget(lbl_name, base_row, 0)

            lbl_pos = QLabel("pos: (0.000, 0.000, 0.000)")
            lbl_pos.setMinimumWidth(260)
            frame_layout.addWidget(lbl_pos, base_row, 1, 1, 2)

            # 第二行：空白 + rpy
            spacer = QLabel("")  # 对齐占位
            frame_layout.addWidget(spacer, base_row + 1, 0)

            lbl_euler = QLabel("rpy[deg]: (0.0, 0.0, 0.0)")
            lbl_euler.setMinimumWidth(260)
            frame_layout.addWidget(lbl_euler, base_row + 1, 1, 1, 2)

            self.labels_frame[name] = (lbl_pos, lbl_euler)

        # 刷新按钮单独一行
        last_row = len(frames) * 2
        self.btn_refresh_frames = QPushButton("刷新坐标系")
        self.btn_refresh_frames.setMaximumWidth(120)
        frame_layout.addWidget(self.btn_refresh_frames, last_row, 0, 1, 1)

        left_layout.addWidget(frame_group)

        # ---------------- 左列：坐标变换测试 ----------------
        transform_group = QGroupBox("坐标变换测试")
        t_layout = QGridLayout(transform_group)
        t_layout.setHorizontalSpacing(6)
        t_layout.setVerticalSpacing(4)
        t_layout.setContentsMargins(8, 8, 8, 8)

        # 第 1 行：源/目标坐标系
        t_layout.addWidget(QLabel("源坐标系:"), 0, 0)
        self.combo_from_frame = QComboBox()
        self.combo_from_frame.addItems(["W", "J", "O", "S"])
        self.combo_from_frame.setMaximumWidth(80)
        t_layout.addWidget(self.combo_from_frame, 0, 1)

        t_layout.addWidget(QLabel("目标坐标系:"), 0, 2)
        self.combo_to_frame = QComboBox()
        self.combo_to_frame.addItems(["W", "J", "O", "S"])
        self.combo_to_frame.setMaximumWidth(80)
        t_layout.addWidget(self.combo_to_frame, 0, 3)

        # 第 2 行：点 p_src
        t_layout.addWidget(QLabel("点 p_src:"), 1, 0)
        self.spin_px = QDoubleSpinBox()
        self.spin_py = QDoubleSpinBox()
        self.spin_pz = QDoubleSpinBox()
        for sp in (self.spin_px, self.spin_py, self.spin_pz):
            sp.setRange(-1000.0, 1000.0)
            sp.setDecimals(3)
            sp.setSingleStep(0.1)
            sp.setMaximumWidth(90)
        t_layout.addWidget(self.spin_px, 1, 1)
        t_layout.addWidget(self.spin_py, 1, 2)
        t_layout.addWidget(self.spin_pz, 1, 3)

        self.btn_transform_point = QPushButton("转换点")
        self.btn_transform_point.setMaximumWidth(80)
        t_layout.addWidget(self.btn_transform_point, 1, 4)

        # 第 3 行：方向 v_src
        t_layout.addWidget(QLabel("方向 v_src:"), 2, 0)
        self.spin_vx = QDoubleSpinBox()
        self.spin_vy = QDoubleSpinBox()
        self.spin_vz = QDoubleSpinBox()
        for sp in (self.spin_vx, self.spin_vy, self.spin_vz):
            sp.setRange(-10.0, 10.0)
            sp.setDecimals(3)
            sp.setSingleStep(0.1)
            sp.setMaximumWidth(90)
        self.spin_vz.setValue(1.0)   # 默认 (0,0,1)

        t_layout.addWidget(self.spin_vx, 2, 1)
        t_layout.addWidget(self.spin_vy, 2, 2)
        t_layout.addWidget(self.spin_vz, 2, 3)

        self.btn_transform_dir = QPushButton("转换方向")
        self.btn_transform_dir.setMaximumWidth(80)
        t_layout.addWidget(self.btn_transform_dir, 2, 4)

        left_layout.addWidget(transform_group)
        left_layout.addStretch(1)

        # ---------------- 右列：关节状态与规划 ----------------
        joint_group = QGroupBox("UR5 + 转台 关节状态与规划")
        j_layout = QGridLayout(joint_group)
        j_layout.setHorizontalSpacing(6)
        j_layout.setVerticalSpacing(3)
        j_layout.setContentsMargins(8, 8, 8, 8)

        self.labels_joint_curr = {}
        self.spin_target_joints = {}

        joint_names = [f"joint{i}" for i in range(1, 7)] + ["turtle_joint"]

        # 表头：关节 / 当前 / 目标
        j_layout.addWidget(QLabel("关节"), 0, 0)
        j_layout.addWidget(QLabel("当前[deg]"), 0, 1)
        j_layout.addWidget(QLabel("目标[deg]"), 0, 2)

        for idx, name in enumerate(joint_names):
            row = idx + 1
            j_layout.addWidget(QLabel(name + ":"), row, 0)

            lbl_curr = QLabel("0.0")
            lbl_curr.setMinimumWidth(60)
            self.labels_joint_curr[name] = lbl_curr
            j_layout.addWidget(lbl_curr, row, 1)

            sp = QDoubleSpinBox()
            sp.setRange(-360.0, 360.0)
            sp.setDecimals(2)
            sp.setSingleStep(5.0)
            sp.setMaximumWidth(90)
            sp.setValue(0.0)
            self.spin_target_joints[name] = sp
            j_layout.addWidget(sp, row, 2)

        # 右侧控制列
        col_btn = 3
        self.btn_refresh_joints = QPushButton("刷新关节")
        self.btn_refresh_joints.setMaximumWidth(110)
        j_layout.addWidget(self.btn_refresh_joints, 0, col_btn)

        self.btn_go_home = QPushButton("回 Home")
        self.btn_go_home.setMaximumWidth(110)
        j_layout.addWidget(self.btn_go_home, 1, col_btn)

        j_layout.addWidget(QLabel("总 yaw 目标[deg]:"), 2, col_btn)
        self.spin_total_yaw_deg = QDoubleSpinBox()
        self.spin_total_yaw_deg.setRange(-360.0, 360.0)
        self.spin_total_yaw_deg.setSingleStep(5.0)
        self.spin_total_yaw_deg.setValue(150.0)
        self.spin_total_yaw_deg.setMaximumWidth(90)
        j_layout.addWidget(self.spin_total_yaw_deg, 3, col_btn)

        self.btn_go_yaw = QPushButton("执行 yaw 分配")
        self.btn_go_yaw.setMaximumWidth(110)
        j_layout.addWidget(self.btn_go_yaw, 4, col_btn)

        self.btn_move_to_target = QPushButton("运动到目标配置")
        self.btn_move_to_target.setMaximumWidth(130)
        j_layout.addWidget(self.btn_move_to_target, 5, col_btn)

        right_layout.addWidget(joint_group)

        # ---------------- 右列：关节手动拖动 (JOG) ----------------
        joint_jog_group = QGroupBox("关节 JOG 拖动")
        jog_layout = QGridLayout(joint_jog_group)
        jog_layout.setHorizontalSpacing(6)
        jog_layout.setVerticalSpacing(3)
        jog_layout.setContentsMargins(8, 8, 8, 8)

        self.sliders_joints = {}
        self.labels_jog_value = {}

        for row, name in enumerate(joint_names):
            jog_layout.addWidget(QLabel(name + ":"), row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setSingleStep(1)
            slider.setPageStep(5)
            slider.setValue(0)
            slider.setTickPosition(QSlider.NoTicks)
            slider.setMaximumWidth(260)

            lbl_val = QLabel("0.0°")
            lbl_val.setMinimumWidth(60)

            jog_layout.addWidget(slider, row, 1, 1, 2)
            jog_layout.addWidget(lbl_val, row, 3)

            self.sliders_joints[name] = slider
            self.labels_jog_value[name] = lbl_val

        right_layout.addWidget(joint_jog_group)
        right_layout.addStretch(1)

        main_layout.addStretch(1)

    def _connect_slots(self):
        # 通讯相关
        self.btn_connect.clicked.connect(self.slots.on_connect_clicked)
        self.btn_disconnect.clicked.connect(self.slots.on_disconnect_clicked)
        self.btn_refresh_frames.clicked.connect(self.slots.on_refresh_frames_clicked)
        self.btn_transform_point.clicked.connect(self.slots.on_transform_point_clicked)
        self.btn_transform_dir.clicked.connect(self.slots.on_transform_dir_clicked)

        # 关节与规划相关
        self.btn_refresh_joints.clicked.connect(self.slots.on_refresh_joints_clicked)
        self.btn_go_home.clicked.connect(self.slots.on_go_home_clicked)
        self.btn_move_to_target.clicked.connect(self.slots.on_move_to_target_clicked)
        self.btn_go_yaw.clicked.connect(self.slots.on_go_yaw_clicked)

        # 关节拖动（jog）
        for name, slider in self.sliders_joints.items():
            slider.valueChanged.connect(
                lambda val, n=name: self.slots.on_joint_slider_changed(n, val)
            )

    # ----------------- 工具：更新 UI 显示 -----------------

    def set_status_text(self, text: str):
        self.label_status.setText(text)

    def on_connected(self, comm: RobotComm):
        self.comm = comm
        self.robot_model = RobotModel()
        self.robot_planner = RobotPlanner(self.robot_model, self.comm)

        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.set_status_text("已连接")
        print("[RobotMainWidget] Connected.")

    def on_disconnected(self):
        if self.comm is not None:
            try:
                self.comm.close()
            except Exception as e:
                print("[RobotMainWidget] 关闭通讯异常:", e)

        self.comm = None
        self.robot_model = None
        self.robot_planner = None

        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.set_status_text("未连接")
        print("[RobotMainWidget] Disconnected.")

    def update_frames_display(self):
        if self.comm is None or not self.comm.is_connected():
            self.set_status_text("未连接，无法刷新坐标系")
            return

        frames = {
            "W": self.comm.get_T_W(),
            "J": self.comm.get_T_WJ(),
            "O": self.comm.get_T_WO(),
            "S": self.comm.get_T_WS(),
            "B": self.comm.get_T_WB(),
        }

        for name, T in frames.items():
            lbl_pos, lbl_euler = self.labels_frame[name]
            t, e = self.comm._matrix_to_pos_euler(T)
            e_deg = e * 180.0 / np.pi
            lbl_pos.setText(
                f"pos: ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})"
            )
            lbl_euler.setText(
                f"rpy[deg]: ({e_deg[0]:.1f}, {e_deg[1]:.1f}, {e_deg[2]:.1f})"
            )

    def update_joint_display(self):
        if self.robot_planner is None or self.robot_model is None:
            return

        q_ur5, q_turtle = self.robot_planner.read_current_config()
        deg_ur5 = [np.degrees(v) for v in q_ur5]
        deg_turtle = float(np.degrees(q_turtle))

        for i, name in enumerate(self.robot_model.ur5_joint_names):
            lbl = self.labels_joint_curr.get(name)
            if lbl is not None:
                lbl.setText(f"{deg_ur5[i]:.2f}")

        lbl_t = self.labels_joint_curr.get(self.robot_model.turtle_joint_name)
        if lbl_t is not None:
            lbl_t.setText(f"{deg_turtle:.2f}")

        # 同步 jog 滑条与显示
        for i, name in enumerate(self.robot_model.ur5_joint_names):
            if name in self.sliders_joints:
                val_deg = deg_ur5[i]
                self.sliders_joints[name].blockSignals(True)
                self.sliders_joints[name].setValue(int(round(val_deg)))
                self.sliders_joints[name].blockSignals(False)
                self.labels_jog_value[name].setText(f"{val_deg:.1f}°")

        tname = self.robot_model.turtle_joint_name
        if tname in self.sliders_joints:
            self.sliders_joints[tname].blockSignals(True)
            self.sliders_joints[tname].setValue(int(round(deg_turtle)))
            self.sliders_joints[tname].blockSignals(False)
            self.labels_jog_value[tname].setText(f"{deg_turtle:.1f}°")

    def closeEvent(self, event):
        if self.comm is not None:
            try:
                self.comm.close()
            except Exception as e:
                print("[RobotMainWidget] 关闭通讯异常:", e)
        event.accept()
