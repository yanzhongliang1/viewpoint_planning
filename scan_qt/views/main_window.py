# scan_qt/views/main_window.py
import os
import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
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

        # ---- 基本窗口属性 ----
        self.setWindowTitle("NBV 控制面板（MVC）")
        # 固定窗口大小，避免拉伸导致布局变形
        self.setFixedSize(1200, 700)

        # 简单工业风深色样式
        self.setStyleSheet("""
                    QMainWindow {
                        background-color: #2b2b2b;
                        color: #dddddd;
                    }
                    QWidget {
                        background-color: #2b2b2b;
                        color: #dddddd;
                        font-size: 10pt;
                    }
                    QGroupBox {
                        border: 1px solid #555555;
                        border-radius: 4px;
                        margin-top: 8px;
                    }
                    QGroupBox::title {
                        subcontrol-origin: margin;
                        left: 8px;
                        padding: 0 3px 0 3px;
                    }
                    QLabel {
                        color: #dddddd;
                    }
                    QCheckBox, QRadioButton {
                        color: #dddddd;
                    }
                    QPushButton {
                        background-color: #444444;
                        border: 1px solid #666666;
                        border-radius: 3px;
                        padding: 4px 10px;
                    }
                    QPushButton:hover {
                        background-color: #555555;
                    }
                    QPushButton:pressed {
                        background-color: #333333;
                    }
                    QSpinBox, QDoubleSpinBox, QComboBox {
                        background-color: #3b3b3b;
                        color: #dddddd;
                        border: 1px solid #555555;
                        border-radius: 2px;
                    }
                    QTableWidget {
                        background-color: #3b3b3b;
                        color: #dddddd;
                        gridline-color: #555555;
                    }
                    QHeaderView::section {
                        background-color: #3b3b3b;
                        color: #dddddd;
                        padding: 3px;
                        border: 1px solid #555555;
                    }
                """)

        # ---- MVC 组件 ----
        self.model = ModelModel()
        self.view3d = ModelView()
        self.controller = ModelController(self.model, self.view3d)

        self.camera_model = CameraModel()
        self.camera_controller = CameraController(self.model, self.view3d, self.camera_model)

        # ---- 中心布局：左右两列 ----
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        left_layout = QVBoxLayout()  # 左列：文件 + 显示 + 视点/相机
        right_layout = QVBoxLayout()  # 右列：点云处理 + 扫描帧管理

        main_layout.addLayout(left_layout, 2)
        main_layout.addLayout(right_layout, 3)

        # =========================================================
        # 左列上：文件操作
        # =========================================================
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

        left_layout.addWidget(file_group)

        # =========================================================
        # 左列中：显示与包围盒 + 视点切换
        # =========================================================
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

        # 视点按钮（前后左右上下）
        view_group = QGroupBox("视点切换")
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
            btn.setMinimumWidth(55)
            btn.clicked.connect(lambda _, n=name: self.change_view(n))
            view_layout.addWidget(btn)

        left_layout.addWidget(display_group)
        left_layout.addWidget(view_group)

        # =========================================================
        # 左列下：扫描仪 / Camera 参数
        # =========================================================
        # ---------------- 扫描仪 / Camera 设置 ----------------
        camera_group = QGroupBox("扫描仪 / Camera 参数")
        cam_main_layout = QHBoxLayout(camera_group)
        cam_main_layout.setContentsMargins(6, 6, 6, 6)
        cam_main_layout.setSpacing(8)

        # 左侧：参数网格
        param_layout = QGridLayout()
        param_layout.setHorizontalSpacing(6)
        param_layout.setVerticalSpacing(4)

        # FOV / Near / Far / Best distance
        lbl_fov = QLabel("FOV(°):")
        self.spin_cam_fov = QDoubleSpinBox()
        self.spin_cam_fov.setRange(1.0, 179.0)
        self.spin_cam_fov.setSingleStep(1.0)
        self.spin_cam_fov.setValue(self.camera_model.fov_deg)
        self.spin_cam_fov.setMaximumWidth(80)

        lbl_near = QLabel("Near:")
        self.spin_cam_near = QDoubleSpinBox()
        self.spin_cam_near.setDecimals(3)
        self.spin_cam_near.setRange(0.001, 100.0)
        self.spin_cam_near.setSingleStep(0.01)
        self.spin_cam_near.setValue(self.camera_model.near)
        self.spin_cam_near.setMaximumWidth(80)

        lbl_far = QLabel("Far:")
        self.spin_cam_far = QDoubleSpinBox()
        self.spin_cam_far.setDecimals(3)
        self.spin_cam_far.setRange(0.01, 1000.0)
        self.spin_cam_far.setSingleStep(0.1)
        self.spin_cam_far.setValue(self.camera_model.far)
        self.spin_cam_far.setMaximumWidth(80)

        lbl_best = QLabel("最佳距离:")
        self.spin_cam_best = QDoubleSpinBox()
        self.spin_cam_best.setDecimals(3)
        self.spin_cam_best.setRange(0.01, 1000.0)
        self.spin_cam_best.setSingleStep(0.1)
        self.spin_cam_best.setValue(self.camera_model.best_distance)
        self.spin_cam_best.setMaximumWidth(80)

        # Pos / Dir 一行两个参数
        lbl_posx = QLabel("PosX:")
        self.spin_cam_posx = QDoubleSpinBox()
        self.spin_cam_posx.setRange(-1000, 1000)
        self.spin_cam_posx.setSingleStep(0.1)
        self.spin_cam_posx.setValue(self.camera_model.position[0])
        self.spin_cam_posx.setMaximumWidth(80)

        lbl_posy = QLabel("PosY:")
        self.spin_cam_posy = QDoubleSpinBox()
        self.spin_cam_posy.setRange(-1000, 1000)
        self.spin_cam_posy.setSingleStep(0.1)
        self.spin_cam_posy.setValue(self.camera_model.position[1])
        self.spin_cam_posy.setMaximumWidth(80)

        lbl_posz = QLabel("PosZ:")
        self.spin_cam_posz = QDoubleSpinBox()
        self.spin_cam_posz.setRange(-1000, 1000)
        self.spin_cam_posz.setSingleStep(0.1)
        self.spin_cam_posz.setValue(self.camera_model.position[2])
        self.spin_cam_posz.setMaximumWidth(80)

        lbl_dirx = QLabel("DirX:")
        self.spin_cam_dirx = QDoubleSpinBox()
        self.spin_cam_dirx.setRange(-1.0, 1.0)
        self.spin_cam_dirx.setSingleStep(0.1)
        self.spin_cam_dirx.setValue(self.camera_model.direction[0])
        self.spin_cam_dirx.setMaximumWidth(80)

        lbl_diry = QLabel("DirY:")
        self.spin_cam_diry = QDoubleSpinBox()
        self.spin_cam_diry.setRange(-1.0, 1.0)
        self.spin_cam_diry.setSingleStep(0.1)
        self.spin_cam_diry.setValue(self.camera_model.direction[1])
        self.spin_cam_diry.setMaximumWidth(80)

        lbl_dirz = QLabel("DirZ:")
        self.spin_cam_dirz = QDoubleSpinBox()
        self.spin_cam_dirz.setRange(-1.0, 1.0)
        self.spin_cam_dirz.setSingleStep(0.1)
        self.spin_cam_dirz.setValue(self.camera_model.direction[2])
        self.spin_cam_dirz.setMaximumWidth(80)

        # 把这些控件丢到网格里
        row = 0
        param_layout.addWidget(lbl_fov, row, 0)
        param_layout.addWidget(self.spin_cam_fov, row, 1)
        param_layout.addWidget(lbl_near, row, 2)
        param_layout.addWidget(self.spin_cam_near, row, 3)

        row += 1
        param_layout.addWidget(lbl_far, row, 0)
        param_layout.addWidget(self.spin_cam_far, row, 1)
        param_layout.addWidget(lbl_best, row, 2)
        param_layout.addWidget(self.spin_cam_best, row, 3)

        row += 1
        param_layout.addWidget(lbl_posx, row, 0)
        param_layout.addWidget(self.spin_cam_posx, row, 1)
        param_layout.addWidget(lbl_posy, row, 2)
        param_layout.addWidget(self.spin_cam_posy, row, 3)

        row += 1
        param_layout.addWidget(lbl_posz, row, 0)
        param_layout.addWidget(self.spin_cam_posz, row, 1)
        param_layout.addWidget(lbl_dirx, row, 2)
        param_layout.addWidget(self.spin_cam_dirx, row, 3)

        row += 1
        param_layout.addWidget(lbl_diry, row, 0)
        param_layout.addWidget(self.spin_cam_diry, row, 1)
        param_layout.addWidget(lbl_dirz, row, 2)
        param_layout.addWidget(self.spin_cam_dirz, row, 3)

        # 左侧网格放进主 layout
        cam_main_layout.addLayout(param_layout, stretch=3)

        # 右侧：操作区（颜色 / 遮挡 / 按钮）
        op_layout = QVBoxLayout()
        op_layout.setSpacing(6)

        color_layout = QHBoxLayout()
        color_layout.addWidget(QLabel("扫描颜色:"))
        self.combo_scan_color = QComboBox()
        self.combo_scan_color.addItems(["自动", "红", "绿", "蓝", "黄", "白"])
        self.combo_scan_color.setMaximumWidth(80)
        color_layout.addWidget(self.combo_scan_color)
        op_layout.addLayout(color_layout)

        self.cb_use_occlusion = QCheckBox("遮挡检测 (ray casting)")
        op_layout.addWidget(self.cb_use_occlusion)

        self.btn_grab_camera = QPushButton("从当前视图获取相机")
        self.btn_grab_camera.clicked.connect(self.on_grab_camera_clicked)
        op_layout.addWidget(self.btn_grab_camera)

        self.btn_update_frustum = QPushButton("更新视锥")
        self.btn_update_frustum.clicked.connect(self.on_update_frustum_clicked)
        op_layout.addWidget(self.btn_update_frustum)

        self.btn_scan_once = QPushButton("扫描一帧")
        self.btn_scan_once.clicked.connect(self.on_scan_once_clicked)
        op_layout.addWidget(self.btn_scan_once)

        op_layout.addStretch(1)
        cam_main_layout.addLayout(op_layout, stretch=2)

        # 最后把 camera_group 放进左侧布局
        left_layout.addWidget(camera_group)

        # =========================================================
        # 右列上：点云处理 + 体素 + 法向
        # =========================================================
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

        right_layout.addWidget(pcd_group)
        right_layout.addWidget(voxel_group)
        right_layout.addWidget(normal_group)

        # =========================================================
        # 右列下：视点 / 扫描帧管理表格
        # =========================================================
        views_group = QGroupBox("视点 / 扫描帧管理")
        views_layout = QVBoxLayout(views_group)

        self.view_table = QTableWidget(0, 3)
        self.view_table.setHorizontalHeaderLabels(["名称", "可见", "颜色"])
        self.view_table.horizontalHeader().setStretchLastSection(True)
        self.view_table.itemDoubleClicked.connect(self.on_view_table_double_clicked)
        views_layout.addWidget(self.view_table)

        self.btn_delete_view = QPushButton("删除选中视点")
        self.btn_delete_view.clicked.connect(self.on_delete_selected_view)
        views_layout.addWidget(self.btn_delete_view)

        self.btn_save_views = QPushButton("保存所有视点")
        self.btn_save_views.clicked.connect(self.on_save_views_clicked)
        views_layout.addWidget(self.btn_save_views)

        self.btn_show_only_selected = QPushButton("只显示选中扫描帧")
        self.btn_show_only_selected.clicked.connect(self.on_show_only_selected_scan)
        views_layout.addWidget(self.btn_show_only_selected)

        self.btn_export_selected_scan = QPushButton("导出选中扫描帧")
        self.btn_export_selected_scan.clicked.connect(self.on_export_selected_scan)
        views_layout.addWidget(self.btn_export_selected_scan)

        self.btn_export_all_scans = QPushButton("导出所有扫描点云")
        self.btn_export_all_scans.clicked.connect(self.on_export_all_scans)
        views_layout.addWidget(self.btn_export_all_scans)

        right_layout.addWidget(views_group, stretch=1)

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

    def on_grab_camera_clicked(self):
        """
        从当前 Open3D 视图读取相机位置/方向，并更新右侧参数框。
        """
        self.camera_controller.grab_pose_from_current_view()

        # 把 camera_model 里的参数写回到界面
        cam = self.camera_model
        self.spin_cam_posx.setValue(cam.position[0])
        self.spin_cam_posy.setValue(cam.position[1])
        self.spin_cam_posz.setValue(cam.position[2])

        self.spin_cam_dirx.setValue(cam.direction[0])
        self.spin_cam_diry.setValue(cam.direction[1])
        self.spin_cam_dirz.setValue(cam.direction[2])

        self.spin_cam_fov.setValue(cam.fov_deg)
        self.spin_cam_near.setValue(cam.near)
        self.spin_cam_far.setValue(cam.far)
        self.spin_cam_best.setValue(cam.best_distance)
        print("UI 更新后的 camera pos:", cam.position)

    def on_view_table_double_clicked(self, item):
        """
        双击某个视点：相机跳转到该视点，并刷新右侧 Camera 参数。
        """
        row = item.row()
        self.camera_controller.go_to_view(row)

        # 用 camera_model 的参数刷新 UI
        cam = self.camera_model
        self.spin_cam_posx.setValue(cam.position[0])
        self.spin_cam_posy.setValue(cam.position[1])
        self.spin_cam_posz.setValue(cam.position[2])
        self.spin_cam_dirx.setValue(cam.direction[0])
        self.spin_cam_diry.setValue(cam.direction[1])
        self.spin_cam_dirz.setValue(cam.direction[2])
        self.spin_cam_fov.setValue(cam.fov_deg)
        self.spin_cam_near.setValue(cam.near)
        self.spin_cam_far.setValue(cam.far)
        self.spin_cam_best.setValue(cam.best_distance)

    def on_save_views_clicked(self):
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "保存视点记录到 txt",
            "",
            "Text 文件 (*.txt)"
        )
        if not filename:
            return
        if not filename.lower().endswith(".txt"):
            filename += ".txt"
        self.camera_controller.save_views_to_txt(filename)

    def on_show_only_selected_scan(self):
        row = self.view_table.currentRow()
        if row < 0:
            return
        # 把所有视点的 visible 先置 False，再只保留当前行可见
        views = self.camera_controller.list_views()
        for i in range(len(views)):
            self.camera_controller.set_view_visibility(i, i == row)
        self.refresh_view_table()

    def on_export_selected_scan(self):
        row = self.view_table.currentRow()
        if row < 0:
            return
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "导出选中扫描帧为 PLY",
            "",
            "PLY 文件 (*.ply)"
        )
        if not filename:
            return
        if not filename.lower().endswith(".ply"):
            filename += ".ply"
        self.camera_controller.export_scan_pcd(row, filename)

    def on_export_all_scans(self):
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "导出所有扫描点云为 PLY",
            "",
            "PLY 文件 (*.ply)"
        )
        if not filename:
            return
        if not filename.lower().endswith(".ply"):
            filename += ".ply"
        self.camera_controller.export_all_scans(filename)
