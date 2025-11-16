# scan_qt/views/main_window.py
import os
import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QFileDialog, QLabel, QGroupBox,
    QCheckBox, QSpinBox, QDoubleSpinBox, QComboBox, QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import QTimer

from scan_qt.models.model_model import ModelModel
from scan_qt.views.model_view import ModelView
from scan_qt.controllers.model_controller import ModelController
from scan_qt.models.camera_model import CameraModel
from scan_qt.controllers.camera_controller import CameraController


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

        # ===== 新增：相机 MVC =====
        self.camera_model = CameraModel()
        self.camera_controller = CameraController(self.model, self.view3d, self.camera_model)
        # ===== 新增结束 =====

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

        # ---------------- 扫描仪 / Camera 设置 ----------------
        camera_group = QGroupBox("扫描仪 / Camera")
        camera_layout = QHBoxLayout(camera_group)

        # FOV
        camera_layout.addWidget(QLabel("FOV(°):"))
        self.spin_cam_fov = QDoubleSpinBox()
        self.spin_cam_fov.setRange(1.0, 179.0)
        self.spin_cam_fov.setSingleStep(1.0)
        self.spin_cam_fov.setValue(self.camera_model.fov_deg)
        camera_layout.addWidget(self.spin_cam_fov)

        # near / far
        camera_layout.addWidget(QLabel("Near:"))
        self.spin_cam_near = QDoubleSpinBox()
        self.spin_cam_near.setDecimals(3)
        self.spin_cam_near.setRange(0.001, 100.0)
        self.spin_cam_near.setSingleStep(0.01)
        self.spin_cam_near.setValue(self.camera_model.near)
        camera_layout.addWidget(self.spin_cam_near)

        camera_layout.addWidget(QLabel("Far:"))
        self.spin_cam_far = QDoubleSpinBox()
        self.spin_cam_far.setDecimals(3)
        self.spin_cam_far.setRange(0.01, 1000.0)
        self.spin_cam_far.setSingleStep(0.1)
        self.spin_cam_far.setValue(self.camera_model.far)
        camera_layout.addWidget(self.spin_cam_far)

        # best distance
        camera_layout.addWidget(QLabel("最佳距离:"))
        self.spin_cam_best = QDoubleSpinBox()
        self.spin_cam_best.setDecimals(3)
        self.spin_cam_best.setRange(0.01, 1000.0)
        self.spin_cam_best.setSingleStep(0.1)
        self.spin_cam_best.setValue(self.camera_model.best_distance)
        camera_layout.addWidget(self.spin_cam_best)

        # 相机位置 pos (x, y, z)
        camera_layout.addWidget(QLabel("PosX:"))
        self.spin_cam_posx = QDoubleSpinBox()
        self.spin_cam_posx.setRange(-1000, 1000)
        self.spin_cam_posx.setSingleStep(0.1)
        self.spin_cam_posx.setValue(self.camera_model.position[0])
        camera_layout.addWidget(self.spin_cam_posx)

        camera_layout.addWidget(QLabel("PosY:"))
        self.spin_cam_posy = QDoubleSpinBox()
        self.spin_cam_posy.setRange(-1000, 1000)
        self.spin_cam_posy.setSingleStep(0.1)
        self.spin_cam_posy.setValue(self.camera_model.position[1])
        camera_layout.addWidget(self.spin_cam_posy)

        camera_layout.addWidget(QLabel("PosZ:"))
        self.spin_cam_posz = QDoubleSpinBox()
        self.spin_cam_posz.setRange(-1000, 1000)
        self.spin_cam_posz.setSingleStep(0.1)
        self.spin_cam_posz.setValue(self.camera_model.position[2])
        camera_layout.addWidget(self.spin_cam_posz)

        # 朝向 dir (dx, dy, dz)
        camera_layout.addWidget(QLabel("DirX:"))
        self.spin_cam_dirx = QDoubleSpinBox()
        self.spin_cam_dirx.setRange(-1.0, 1.0)
        self.spin_cam_dirx.setSingleStep(0.1)
        self.spin_cam_dirx.setValue(self.camera_model.direction[0])
        camera_layout.addWidget(self.spin_cam_dirx)

        camera_layout.addWidget(QLabel("DirY:"))
        self.spin_cam_diry = QDoubleSpinBox()
        self.spin_cam_diry.setRange(-1.0, 1.0)
        self.spin_cam_diry.setSingleStep(0.1)
        self.spin_cam_diry.setValue(self.camera_model.direction[1])
        camera_layout.addWidget(self.spin_cam_diry)

        camera_layout.addWidget(QLabel("DirZ:"))
        self.spin_cam_dirz = QDoubleSpinBox()
        self.spin_cam_dirz.setRange(-1.0, 1.0)
        self.spin_cam_dirz.setSingleStep(0.1)
        self.spin_cam_dirz.setValue(self.camera_model.direction[2])
        camera_layout.addWidget(self.spin_cam_dirz)

        # 扫描点云颜色
        camera_layout.addWidget(QLabel("扫描颜色:"))
        self.combo_scan_color = QComboBox()
        self.combo_scan_color.addItems(["红", "绿", "蓝", "黄", "白"])
        camera_layout.addWidget(self.combo_scan_color)

        # 是否启用遮挡检测
        self.cb_use_occlusion = QCheckBox("遮挡检测(ray casting)")
        self.cb_use_occlusion.setChecked(True)
        camera_layout.addWidget(self.cb_use_occlusion)

        # 按钮：更新视锥 / 扫描一帧
        self.btn_update_frustum = QPushButton("更新视锥")
        self.btn_update_frustum.clicked.connect(self.on_update_frustum_clicked)
        camera_layout.addWidget(self.btn_update_frustum)

        self.btn_scan_once = QPushButton("扫描一帧")
        self.btn_scan_once.clicked.connect(self.on_scan_once_clicked)
        camera_layout.addWidget(self.btn_scan_once)

        # ---------------- 视点管理列表 ----------------
        views_group = QGroupBox("视点 / 扫描帧管理")
        views_layout = QVBoxLayout(views_group)

        self.view_table = QTableWidget(0, 3)
        self.view_table.setHorizontalHeaderLabels(["名称", "可见", "颜色"])
        self.view_table.horizontalHeader().setStretchLastSection(True)
        views_layout.addWidget(self.view_table)

        # 删除选中视点按钮
        self.btn_delete_view = QPushButton("删除选中视点")
        self.btn_delete_view.clicked.connect(self.on_delete_selected_view)
        views_layout.addWidget(self.btn_delete_view)

        # ---------------- 布局汇总 ----------------
        main_layout.addWidget(file_group)
        main_layout.addWidget(display_group)
        main_layout.addWidget(pcd_group)
        main_layout.addWidget(voxel_group)
        main_layout.addWidget(normal_group)
        main_layout.addWidget(camera_group)  # 新增：扫描仪设置
        main_layout.addWidget(views_group)  # 新增视点管理
        main_layout.addWidget(view_group)

        # ---------------- 定时更新 Open3D ----------------
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.controller.update_view)
        self.timer.start(16)
        self.refresh_view_table()

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

    def _get_scan_color_from_combo(self):
        text = self.combo_scan_color.currentText()
        if text == "红":
            return (1.0, 0.0, 0.0)
        elif text == "绿":
            return (0.0, 1.0, 0.0)
        elif text == "蓝":
            return (0.0, 0.0, 1.0)
        elif text == "黄":
            return (1.0, 1.0, 0.0)
        elif text == "白":
            return (1.0, 1.0, 1.0)
        elif text == "自动":
            return None  # 交给 controller 的颜色循环策略
        else:
            return None

    def on_update_frustum_clicked(self):
        # 从 UI 读参数写入 camera_model
        fov = self.spin_cam_fov.value()
        near = self.spin_cam_near.value()
        far = self.spin_cam_far.value()
        best = self.spin_cam_best.value()

        pos = (
            self.spin_cam_posx.value(),
            self.spin_cam_posy.value(),
            self.spin_cam_posz.value(),
        )
        direction = (
            self.spin_cam_dirx.value(),
            self.spin_cam_diry.value(),
            self.spin_cam_dirz.value(),
        )

        self.camera_controller.set_intrinsics(
            fov_deg=fov, near=near, far=far, best_distance=best
        )
        self.camera_controller.set_pose(pos=np.array(pos), direction=np.array(direction))

        # 不强制新增视点，只是更新内部相机参数 &（若你愿意）预览视锥
        self.camera_controller.update_frustum()

    def on_scan_once_clicked(self):
        color = self._get_scan_color_from_combo()
        use_occ = self.cb_use_occlusion.isChecked()
        self.camera_controller.scan_once(color=color, use_occlusion=use_occ)
        self.refresh_view_table()

    def refresh_view_table(self):
        views = self.camera_controller.list_views()
        self.view_table.blockSignals(True)  # 避免刷新时触发信号
        self.view_table.setRowCount(len(views))

        for row, info in enumerate(views):
            # 名称
            item_name = QTableWidgetItem(info["name"])
            self.view_table.setItem(row, 0, item_name)

            # 可见性复选框
            cb = QCheckBox()
            cb.setChecked(info["visible"])
            # 绑定行号
            cb.stateChanged.connect(lambda state, r=row: self.on_view_visibility_changed(r, state))
            self.view_table.setCellWidget(row, 1, cb)

            # 颜色文本
            color = info["color"]
            color_text = f"({color[0]:.2f},{color[1]:.2f},{color[2]:.2f})"
            item_color = QTableWidgetItem(color_text)
            self.view_table.setItem(row, 2, item_color)

        self.view_table.blockSignals(False)

    def on_view_visibility_changed(self, row, state):
        visible = bool(state)
        self.camera_controller.set_view_visibility(row, visible)

    def on_delete_selected_view(self):
        row = self.view_table.currentRow()
        if row < 0:
            return
        self.camera_controller.remove_view(row)
        self.refresh_view_table()
