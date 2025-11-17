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

from scan_qt.views.main_window_slots import MainWindowSlots


class MainWindow(QMainWindow):
    """
    View（Qt）：只负责 UI，槽函数放在 MainWindowSlots 里。
    """

    def __init__(self):
        super().__init__()

        # ---- 基本窗口属性 ----
        self.setWindowTitle("NBV 控制面板（MVC）")
        self.setFixedSize(1200, 700)

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

        # 槽函数管理类
        self.slots = MainWindowSlots(self)

        # ---- 中心布局：左右两列 ----
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        left_layout = QVBoxLayout()   # 左列：文件 + 显示 + 视点/相机
        right_layout = QVBoxLayout()  # 右列：点云处理 + 扫描帧管理

        main_layout.addLayout(left_layout, 2)
        main_layout.addLayout(right_layout, 3)

        # =========================================================
        # 左列上：文件操作
        # =========================================================
        file_group = QGroupBox("文件操作")
        file_layout = QHBoxLayout(file_group)

        self.open_btn = QPushButton("打开 PLY")
        self.open_btn.clicked.connect(self.slots.open_file)

        self.save_btn = QPushButton("保存当前为 PLY")
        self.save_btn.clicked.connect(self.slots.save_file)

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
        self.cb_show_bbox.stateChanged.connect(self.slots.on_bbox_toggled)
        display_layout.addWidget(self.cb_show_bbox)

        display_layout.addWidget(QLabel("点大小:"))
        self.spin_point_size = QDoubleSpinBox()
        self.spin_point_size.setRange(1.0, 20.0)
        self.spin_point_size.setSingleStep(0.5)
        self.spin_point_size.setValue(2.0)
        self.spin_point_size.valueChanged.connect(self.slots.on_point_size_changed)
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
            btn.clicked.connect(lambda _, n=name: self.slots.change_view(n))
            view_layout.addWidget(btn)

        left_layout.addWidget(display_group)
        left_layout.addWidget(view_group)

        # =========================================================
        # 左列下：扫描仪 / Camera 参数
        # =========================================================
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

        # Pos / Dir
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

        # 放进网格
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

        cam_main_layout.addLayout(param_layout, stretch=3)

        # 右侧操作区
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
        self.btn_grab_camera.clicked.connect(self.slots.on_grab_camera_clicked)
        op_layout.addWidget(self.btn_grab_camera)

        self.btn_update_frustum = QPushButton("更新视锥")
        self.btn_update_frustum.clicked.connect(self.slots.on_update_frustum_clicked)
        op_layout.addWidget(self.btn_update_frustum)

        self.btn_scan_once = QPushButton("扫描一帧")
        self.btn_scan_once.clicked.connect(self.slots.on_scan_once_clicked)
        op_layout.addWidget(self.btn_scan_once)

        op_layout.addStretch(1)
        cam_main_layout.addLayout(op_layout, stretch=2)

        left_layout.addWidget(camera_group)

        # =========================================================
        # 右列上：点云处理 + 体素 + 法向
        # =========================================================
        pcd_group = QGroupBox("点云处理")
        pcd_layout = QHBoxLayout(pcd_group)

        self.cb_use_pcd = QCheckBox("点云模式(网格采样)")
        self.cb_use_pcd.stateChanged.connect(self.slots.on_use_pcd_toggled)
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
        self.btn_apply_downsample.clicked.connect(self.slots.on_apply_downsample)
        pcd_layout.addWidget(self.btn_apply_downsample)

        voxel_group = QGroupBox("体素分割显示")
        voxel_layout = QHBoxLayout(voxel_group)

        self.cb_show_voxel = QCheckBox("显示体素网格")
        self.cb_show_voxel.stateChanged.connect(self.slots.on_voxel_toggled)
        voxel_layout.addWidget(self.cb_show_voxel)

        voxel_layout.addWidget(QLabel("体素大小:"))
        self.spin_voxel_size = QDoubleSpinBox()
        self.spin_voxel_size.setDecimals(5)
        self.spin_voxel_size.setRange(1e-5, 1.0)
        self.spin_voxel_size.setSingleStep(0.001)
        self.spin_voxel_size.setValue(0.01)
        self.spin_voxel_size.valueChanged.connect(self.slots.on_voxel_size_changed)
        voxel_layout.addWidget(self.spin_voxel_size)

        normal_group = QGroupBox("法向显示")
        normal_layout = QHBoxLayout(normal_group)

        self.cb_show_normals = QCheckBox("显示法向")
        self.cb_show_normals.stateChanged.connect(self.slots.on_show_normals_toggled)
        normal_layout.addWidget(self.cb_show_normals)

        normal_layout.addWidget(QLabel("法向长度:"))
        self.spin_normal_length = QDoubleSpinBox()
        self.spin_normal_length.setDecimals(5)
        self.spin_normal_length.setRange(1e-4, 1.0)
        self.spin_normal_length.setSingleStep(0.001)
        self.spin_normal_length.setValue(0.01)
        self.spin_normal_length.valueChanged.connect(self.slots.on_normal_params_changed)
        normal_layout.addWidget(self.spin_normal_length)

        normal_layout.addWidget(QLabel("采样步长:"))
        self.spin_normal_step = QSpinBox()
        self.spin_normal_step.setRange(1, 1000)
        self.spin_normal_step.setSingleStep(1)
        self.spin_normal_step.setValue(10)
        self.spin_normal_step.valueChanged.connect(self.slots.on_normal_params_changed)
        normal_layout.addWidget(self.spin_normal_step)

        normal_layout.addWidget(QLabel("颜色:"))
        self.combo_normal_color = QComboBox()
        self.combo_normal_color.addItems(["红", "绿", "蓝", "白"])
        self.combo_normal_color.currentIndexChanged.connect(self.slots.on_normal_params_changed)
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
        self.view_table.itemDoubleClicked.connect(self.slots.on_view_table_double_clicked)
        views_layout.addWidget(self.view_table)

        self.btn_delete_view = QPushButton("删除选中视点")
        self.btn_delete_view.clicked.connect(self.slots.on_delete_selected_view)
        views_layout.addWidget(self.btn_delete_view)

        self.btn_save_views = QPushButton("保存所有视点")
        self.btn_save_views.clicked.connect(self.slots.on_save_views_clicked)
        views_layout.addWidget(self.btn_save_views)

        self.btn_show_only_selected = QPushButton("只显示选中扫描帧")
        self.btn_show_only_selected.clicked.connect(self.slots.on_show_only_selected_scan)
        views_layout.addWidget(self.btn_show_only_selected)

        self.btn_export_selected_scan = QPushButton("导出选中扫描帧")
        self.btn_export_selected_scan.clicked.connect(self.slots.on_export_selected_scan)
        views_layout.addWidget(self.btn_export_selected_scan)

        self.btn_export_all_scans = QPushButton("导出所有扫描点云")
        self.btn_export_all_scans.clicked.connect(self.slots.on_export_all_scans)
        views_layout.addWidget(self.btn_export_all_scans)

        right_layout.addWidget(views_group, stretch=1)

        # ---------------- 定时更新 Open3D ----------------
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.controller.update_view)
        self.timer.start(16)

        # 初始化视点表格
        self.slots.refresh_view_table()

