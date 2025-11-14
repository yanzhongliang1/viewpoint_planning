# scan_qt/views/main_window.py
import os

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QFileDialog, QLabel, QGroupBox,
    QCheckBox, QSpinBox, QDoubleSpinBox, QComboBox
)
from PyQt5.QtCore import QTimer

from scan_qt.models.model_model import ModelModel
from scan_qt.views.model_view import ModelView
from scan_qt.controllers.model_controller import ModelController


class MainWindow(QMainWindow):
    """
    View（Qt）：只负责 UI，所有逻辑都丢给 controller。
    """

    def __init__(self):
        super().__init__()

        self.setWindowTitle("NBV 控制面板（MVC）")
        self.resize(800, 400)

        # MVC 组件
        self.model = ModelModel()
        self.view3d = ModelView()
        self.controller = ModelController(self.model, self.view3d)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ---------------- 文件操作 ----------------
        file_group = QGroupBox("文件操作")
        file_layout = QHBoxLayout(file_group)

        self.open_btn = QPushButton("打开 PLY")
        self.open_btn.clicked.connect(self.open_file)

        self.save_btn = QPushButton("保存当前为 PLY")
        self.save_btn.clicked.connect(self.save_file)

        self.file_label = QLabel("未选择文件")

        file_layout.addWidget(self.open_btn)
        file_layout.addWidget(self.save_btn)
        file_layout.addWidget(self.file_label)

        # ---------------- 显示与包围盒 ----------------
        display_group = QGroupBox("显示与包围盒")
        display_layout = QHBoxLayout(display_group)

        self.cb_show_bbox = QCheckBox("显示包围盒")
        self.cb_show_bbox.stateChanged.connect(self.on_bbox_toggled)
        display_layout.addWidget(self.cb_show_bbox)

        display_layout.addWidget(QLabel("点大小:"))
        self.spin_point_size = QDoubleSpinBox()
        self.spin_point_size.setRange(1.0, 20.0)
        self.spin_point_size.setSingleStep(0.5)
        self.spin_point_size.setValue(2.0)
        self.spin_point_size.valueChanged.connect(self.on_point_size_changed)
        display_layout.addWidget(self.spin_point_size)

        # ---------------- 点云处理 ----------------
        pcd_group = QGroupBox("点云处理")
        pcd_layout = QHBoxLayout(pcd_group)

        self.cb_use_pcd = QCheckBox("点云模式(网格采样)")
        self.cb_use_pcd.stateChanged.connect(self.on_use_pcd_toggled)
        pcd_layout.addWidget(self.cb_use_pcd)

        pcd_layout.addWidget(QLabel("采样点数:"))
        self.spin_sample_points = QSpinBox()
        self.spin_sample_points.setRange(1000, 2_000_000)
        self.spin_sample_points.setSingleStep(10000)
        self.spin_sample_points.setValue(200000)
        pcd_layout.addWidget(self.spin_sample_points)

        pcd_layout.addWidget(QLabel("降采样体素大小:"))
        self.spin_down_voxel = QDoubleSpinBox()
        self.spin_down_voxel.setDecimals(5)
        self.spin_down_voxel.setRange(1e-5, 1.0)
        self.spin_down_voxel.setSingleStep(0.001)
        self.spin_down_voxel.setValue(0.01)
        pcd_layout.addWidget(self.spin_down_voxel)

        self.btn_apply_downsample = QPushButton("应用降采样")
        self.btn_apply_downsample.clicked.connect(self.on_apply_downsample)
        pcd_layout.addWidget(self.btn_apply_downsample)

        # ---------------- 体素分割显示 ----------------
        voxel_group = QGroupBox("体素分割显示")
        voxel_layout = QHBoxLayout(voxel_group)

        self.cb_show_voxel = QCheckBox("显示体素网格")
        self.cb_show_voxel.stateChanged.connect(self.on_voxel_toggled)
        voxel_layout.addWidget(self.cb_show_voxel)

        voxel_layout.addWidget(QLabel("体素大小:"))
        self.spin_voxel_size = QDoubleSpinBox()
        self.spin_voxel_size.setDecimals(5)
        self.spin_voxel_size.setRange(1e-5, 1.0)
        self.spin_voxel_size.setSingleStep(0.001)
        self.spin_voxel_size.setValue(0.01)
        self.spin_voxel_size.valueChanged.connect(self.on_voxel_size_changed)
        voxel_layout.addWidget(self.spin_voxel_size)

        # ---------------- 法向显示 ----------------
        normal_group = QGroupBox("法向显示")
        normal_layout = QHBoxLayout(normal_group)

        self.cb_show_normals = QCheckBox("显示法向")
        self.cb_show_normals.stateChanged.connect(self.on_show_normals_toggled)
        normal_layout.addWidget(self.cb_show_normals)

        normal_layout.addWidget(QLabel("法向长度:"))
        self.spin_normal_length = QDoubleSpinBox()
        self.spin_normal_length.setDecimals(5)
        self.spin_normal_length.setRange(1e-4, 1.0)
        self.spin_normal_length.setSingleStep(0.001)
        self.spin_normal_length.setValue(0.01)
        self.spin_normal_length.valueChanged.connect(self.on_normal_params_changed)
        normal_layout.addWidget(self.spin_normal_length)

        normal_layout.addWidget(QLabel("采样步长:"))
        self.spin_normal_step = QSpinBox()
        self.spin_normal_step.setRange(1, 1000)
        self.spin_normal_step.setSingleStep(1)
        self.spin_normal_step.setValue(10)
        self.spin_normal_step.valueChanged.connect(self.on_normal_params_changed)
        normal_layout.addWidget(self.spin_normal_step)

        normal_layout.addWidget(QLabel("颜色:"))
        self.combo_normal_color = QComboBox()
        self.combo_normal_color.addItems(["红", "绿", "蓝", "白"])
        self.combo_normal_color.currentIndexChanged.connect(self.on_normal_params_changed)
        normal_layout.addWidget(self.combo_normal_color)

        # ---------------- 视点 ----------------
        view_group = QGroupBox("视点设置（预留，可改视角盒）")
        view_layout = QHBoxLayout(view_group)

        for text, name in [
            ("前视", "front"),
            ("后视", "back"),
            ("左视", "left"),
            ("右视", "right"),
            ("上视", "top"),
            ("下视", "bottom"),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, n=name: self.change_view(n))
            view_layout.addWidget(btn)

        # ---------------- 布局汇总 ----------------
        main_layout.addWidget(file_group)
        main_layout.addWidget(display_group)
        main_layout.addWidget(pcd_group)
        main_layout.addWidget(voxel_group)
        main_layout.addWidget(normal_group)
        main_layout.addWidget(view_group)

        # ---------------- 定时更新 Open3D ----------------
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.controller.update_view)
        self.timer.start(16)

    # ----------- 文件操作 -----------

    def open_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "选择 PLY 模型文件",
            "",
            "PLY 文件 (*.ply);;所有文件 (*.*)",
        )
        if filename:
            ok = self.controller.open_ply(filename)
            self.file_label.setText(os.path.basename(filename) if ok else "加载失败")

    def save_file(self):
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "保存当前几何为 PLY",
            "",
            "PLY 文件 (*.ply)"
        )
        if filename:
            if not filename.lower().endswith(".ply"):
                filename += ".ply"
            ok = self.controller.save_ply(filename)
            self.file_label.setText(
                "已保存: " + os.path.basename(filename) if ok else "保存失败"
            )

    # ----------- 显示与包围盒 -----------

    def on_bbox_toggled(self, state):
        self.controller.toggle_bbox(bool(state))

    def on_point_size_changed(self, value):
        self.controller.set_point_size(float(value))

    # ----------- 点云处理 -----------

    def on_use_pcd_toggled(self, state):
        num_points = self.spin_sample_points.value()
        self.controller.toggle_use_sampled_pcd(bool(state), num_points=num_points)

    def on_apply_downsample(self):
        voxel_size = self.spin_down_voxel.value()
        self.controller.apply_voxel_downsample(voxel_size)

    # ----------- 体素分割显示 -----------

    def on_voxel_toggled(self, state):
        self.controller.toggle_voxel_grid(bool(state))

    def on_voxel_size_changed(self, value):
        self.controller.set_voxel_size(float(value))

    # ----------- 法向显示 -----------

    def _get_normal_color_from_combo(self):
        text = self.combo_normal_color.currentText()
        if text == "红":
            return (1.0, 0.0, 0.0)
        elif text == "绿":
            return (0.0, 1.0, 0.0)
        elif text == "蓝":
            return (0.0, 0.0, 1.0)
        elif text == "白":
            return (1.0, 1.0, 1.0)
        else:
            return (1.0, 0.0, 0.0)

    def on_show_normals_toggled(self, state):
        self.controller.toggle_show_normals(bool(state))

    def on_normal_params_changed(self, *args):
        length = self.spin_normal_length.value()
        step = self.spin_normal_step.value()
        color = self._get_normal_color_from_combo()
        self.controller.set_normal_params(length=length, step=step, color=color)

    # ----------- 视点 -----------

    def change_view(self, view_name: str):
        self.controller.set_view(view_name)

    # ----------- 关闭事件 -----------

    def closeEvent(self, event):
        self.controller.close()
        event.accept()
