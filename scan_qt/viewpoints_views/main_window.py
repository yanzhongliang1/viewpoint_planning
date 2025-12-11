# scan_qt\viewpoints_views\main_window.py
import sys
import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QPushButton, QLabel, QGroupBox, QCheckBox, QSpinBox, QDoubleSpinBox,
    QComboBox, QTableWidget, QTableWidgetItem, QSplitter, QTextEdit,
    QTabWidget, QScrollArea, QHeaderView, QFrame, QSizePolicy, QShortcut
)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QIcon, QFont, QKeySequence

from scan_qt.models.model_model import ModelModel
from scan_qt.viewpoints_views.model_view import ModelView
from scan_qt.controllers.model_controller import ModelController
from scan_qt.models.camera_model import CameraModel
from scan_qt.controllers.camera_controller import CameraController
from scan_qt.viewpoints_views.main_window_slots import MainWindowSlots
from scan_qt.controllers.nbv_controller import NBVController
from scan_qt.viewpoints_views.main_window_qss import get_qss
from scan_qt.viewpoints_views.logger import LogWidget

from PyQt5.QtGui import QResizeEvent


# ==== 新增：自定义容器类 (放在 MainWindow 类前面或后面均可) ====
class Open3DContainer(QWidget):
    """
    专门用于容纳 Open3D 窗口的容器。
    重写 resizeEvent 以实现“随动伸缩”。
    """

    def __init__(self, view3d_instance):
        super().__init__()
        self.view3d = view3d_instance
        # 设置策略，确保能接收鼠标事件（防止被 Open3D 完全吞掉，虽然后者在上面）
        self.setFocusPolicy(Qt.StrongFocus)
        self.setStyleSheet("background-color: #1e1e1e;")

    def resizeEvent(self, event: QResizeEvent):
        super().resizeEvent(event)
        # 当这个 Qt 控件大小改变时，命令 Open3D 窗口也改变
        if self.view3d:
            self.view3d.resize_to_container(self)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # ---- 窗口基础设置 ----
        self.setWindowTitle("NBV Scan Studio Pro")
        self.resize(1600, 1000)  # 移除 FixedSize，允许拉伸
        self.setMinimumSize(1280, 800)
        self.setStyleSheet(get_qss())  # 应用样式表

        # ---- MVC 组件 ----
        self.model = ModelModel()
        self.view3d = ModelView()
        self.controller = ModelController(self.model, self.view3d)
        self.camera_model = CameraModel()
        self.camera_controller = CameraController(self.model, self.view3d, self.camera_model)
        self.nbv_controller = NBVController(self.model, self.camera_model, self.view3d)

        self.logger = LogWidget(self)
        self.slots = MainWindowSlots(self)

        # ---- 初始化 UI ----
        self.init_ui()

        # ---- 定时器 ----
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.controller.update_view)
        self.timer.start(16)

        # 初始化数据
        self.slots.refresh_view_table()
        self.logger.log("系统初始化完成。", "INFO")
        QTimer.singleShot(200, self.init_open3d_embedding)

    def init_ui(self):
        """构建主界面布局"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_v_layout = QVBoxLayout(central_widget)
        main_v_layout.setContentsMargins(0, 0, 0, 0)
        main_v_layout.setSpacing(0)

        # 1. 主工作区 (左右分割 + 中间视口)
        self.main_splitter = QSplitter(Qt.Horizontal)
        self.main_splitter.setHandleWidth(2)

        # 左侧面板
        left_panel = self.create_left_panel()
        # 中间视口
        center_panel = self.create_viewport()
        # 右侧面板
        right_panel = self.create_right_panel()

        self.main_splitter.addWidget(left_panel)
        self.main_splitter.addWidget(center_panel)
        self.main_splitter.addWidget(right_panel)

        # 设置初始宽度比例 (左:中:右)
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setStretchFactor(2, 0)
        self.main_splitter.setSizes([320, 900, 320])

        # 2. 底部日志区 (垂直分割)
        self.log_splitter = QSplitter(Qt.Vertical)
        self.log_splitter.setHandleWidth(2)

        self.log_splitter.addWidget(self.main_splitter)
        self.log_splitter.addWidget(self.logger)

        # 设置底部日志初始高度
        self.log_splitter.setStretchFactor(0, 1)
        self.log_splitter.setStretchFactor(1, 0)
        self.log_splitter.setSizes([800, 150])

        main_v_layout.addWidget(self.log_splitter)

    # ================= UI 区域构建 =================

    def create_left_panel(self):
        """左侧：控制面板"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        container = QWidget()
        container.setObjectName("sidePanel")
        layout = QVBoxLayout(container)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # --- A. 文件操作 ---
        grp_file = QGroupBox("项目")
        l_file = QHBoxLayout(grp_file)
        l_file.setContentsMargins(5, 10, 5, 5)

        self.open_btn = QPushButton("导入")
        self.open_btn.setToolTip("导入 PLY 模型")
        self.open_btn.clicked.connect(self.slots.open_file)

        self.save_btn = QPushButton("保存")
        self.save_btn.clicked.connect(self.slots.save_file)

        l_file.addWidget(self.open_btn)
        l_file.addWidget(self.save_btn)
        layout.addWidget(grp_file)

        self.file_label = QLabel("未加载模型")
        self.file_label.setStyleSheet("color: #888; font-size: 11px; margin-bottom: 5px;")
        self.file_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.file_label)

        # --- B. 选项卡分组 ---
        tabs = QTabWidget()
        tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Tab 1: 视图设置
        tab_view = QWidget()
        v_layout = QVBoxLayout(tab_view)

        # 1. 视角快捷键
        g_cam = QGroupBox("快捷视角")
        gl_cam = QGridLayout(g_cam)
        views = [("前视", "front"), ("后视", "back"), ("左视", "left"),
                 ("右视", "right"), ("顶视", "top"), ("底视", "bottom")]
        for i, (txt, key) in enumerate(views):
            btn = QPushButton(txt)
            btn.clicked.connect(lambda _, k=key: self.slots.change_view(k))
            gl_cam.addWidget(btn, i // 2, i % 2)
        v_layout.addWidget(g_cam)

        # 2. 显示选项
        g_disp = QGroupBox("显示选项")
        fl_disp = QFormLayout(g_disp)

        self.cb_show_bbox = QCheckBox()
        self.cb_show_bbox.stateChanged.connect(self.slots.on_bbox_toggled)

        self.cb_show_model_axes = QCheckBox()
        self.cb_show_model_axes.setChecked(True)
        self.cb_show_model_axes.stateChanged.connect(self.slots.on_model_axes_toggled)

        self.spin_point_size = QDoubleSpinBox()
        self.spin_point_size.setRange(0.1, 20.0)
        self.spin_point_size.setValue(2.0)
        self.spin_point_size.valueChanged.connect(self.slots.on_point_size_changed)

        fl_disp.addRow("显示包围盒:", self.cb_show_bbox)
        fl_disp.addRow("显示坐标轴:", self.cb_show_model_axes)
        fl_disp.addRow("全局点大小:", self.spin_point_size)
        v_layout.addWidget(g_disp)
        v_layout.addStretch()

        # Tab 2: 相机与扫描
        tab_scan = QWidget()
        s_layout = QVBoxLayout(tab_scan)

        # Tab 2: 相机与扫描
        tab_scan = QWidget()
        s_layout = QVBoxLayout(tab_scan)

        # --- 新增：表面交互工具 ---
        g_interact = QGroupBox("表面交互")
        l_interact = QVBoxLayout(g_interact)

        self.btn_pick_center = QPushButton("① 拾取中心点 (P)")
        self.btn_pick_center.setToolTip("在屏幕中心发射射线，拾取物体表面的点和法向")
        self.btn_pick_center.clicked.connect(self.slots.on_pick_center_clicked)

        self.btn_align_normal = QPushButton("② 法向对齐视图 (Space)")
        self.btn_align_normal.setToolTip("将相机移动到拾取点的法向正前方")
        self.btn_align_normal.clicked.connect(self.slots.on_align_normal_clicked)

        l_interact.addWidget(self.btn_pick_center)
        l_interact.addWidget(self.btn_align_normal)
        s_layout.addWidget(g_interact)

        # 1. 镜头参数
        g_lens = QGroupBox("镜头参数")
        fl_lens = QFormLayout(g_lens)

        self.spin_cam_fov = self._create_spin(val=self.camera_model.fov_deg, min_v=1, max_v=179)
        self.spin_cam_near = self._create_spin(val=self.camera_model.near, min_v=0.01, max_v=1000)
        self.spin_cam_far = self._create_spin(val=self.camera_model.far, min_v=0.1, max_v=5000)
        self.spin_cam_best = self._create_spin(val=self.camera_model.best_distance, min_v=0.1, max_v=5000)

        fl_lens.addRow("FOV (°):", self.spin_cam_fov)
        fl_lens.addRow("Near / Far:", self._dual_widget(self.spin_cam_near, self.spin_cam_far))
        fl_lens.addRow("最佳距离:", self.spin_cam_best)
        s_layout.addWidget(g_lens)

        # 2. 位姿参数
        g_pose = QGroupBox("相机位姿")
        l_pose = QVBoxLayout(g_pose)

        self.spin_cam_posx = self._create_spin(0, -9999, 9999)
        self.spin_cam_posy = self._create_spin(0, -9999, 9999)
        self.spin_cam_posz = self._create_spin(0, -9999, 9999)
        l_pose.addWidget(QLabel("Position (XYZ):"))
        l_pose.addWidget(self._tri_widget(self.spin_cam_posx, self.spin_cam_posy, self.spin_cam_posz))

        self.spin_cam_dirx = self._create_spin(0, -1, 1, step=0.05)
        self.spin_cam_diry = self._create_spin(0, -1, 1, step=0.05)
        self.spin_cam_dirz = self._create_spin(0, -1, 1, step=0.05)
        l_pose.addWidget(QLabel("Direction (XYZ):"))
        l_pose.addWidget(self._tri_widget(self.spin_cam_dirx, self.spin_cam_diry, self.spin_cam_dirz))

        # 按钮
        h_sync = QHBoxLayout()
        self.btn_grab_camera = QPushButton("同步视图")
        self.btn_grab_camera.clicked.connect(self.slots.on_grab_camera_clicked)
        self.btn_update_frustum = QPushButton("更新视锥")
        self.btn_update_frustum.clicked.connect(self.slots.on_update_frustum_clicked)
        h_sync.addWidget(self.btn_grab_camera)
        h_sync.addWidget(self.btn_update_frustum)
        l_pose.addLayout(h_sync)
        s_layout.addWidget(g_pose)

        # 3. 扫描操作
        g_act = QGroupBox("扫描操作")
        l_act = QVBoxLayout(g_act)

        act_row1 = QHBoxLayout()
        act_row1.addWidget(QLabel("颜色:"))
        self.combo_scan_color = QComboBox()
        self.combo_scan_color.addItems(["自动", "红", "绿", "蓝", "白"])
        act_row1.addWidget(self.combo_scan_color)

        self.cb_use_occlusion = QCheckBox("RayCast 遮挡检测")
        self.cb_use_occlusion.setChecked(False)

        self.btn_scan_once = QPushButton("执行单次扫描")
        self.btn_scan_once.setObjectName("primaryButton")
        self.btn_scan_once.setMinimumHeight(35)
        self.btn_scan_once.clicked.connect(self.slots.on_scan_once_clicked)

        l_act.addLayout(act_row1)
        l_act.addWidget(self.cb_use_occlusion)
        l_act.addWidget(self.btn_scan_once)
        s_layout.addWidget(g_act)
        s_layout.addStretch()

        tabs.addTab(tab_view, "显示")
        tabs.addTab(tab_scan, "相机")
        layout.addWidget(tabs)

        scroll.setWidget(container)
        self._init_shortcuts()
        return scroll

    def _init_shortcuts(self):
        """注册全局快捷键"""
        # P 键拾取
        sc_pick = QShortcut(QKeySequence("P"), self)
        sc_pick.activated.connect(self.slots.on_pick_center_clicked)

        # 空格键对齐
        sc_align = QShortcut(QKeySequence("Space"), self)
        sc_align.activated.connect(self.slots.on_align_normal_clicked)

    def create_viewport(self):
        """中间：3D 视口容器"""
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)

        # --- 核心修改点：使用自定义的 Open3DContainer ---
        # 传入 self.view3d 引用，以便它能调用 resize
        self.view_placeholder = Open3DContainer(self.view3d)

        # 这一步很重要：设置 NativeWindow 属性，产生 WinId
        self.view_placeholder.setAttribute(Qt.WA_NativeWindow)
        self.view_placeholder.setAttribute(Qt.WA_PaintOnScreen)
        self.view_placeholder.setAttribute(Qt.WA_NoSystemBackground)  # 防止闪烁

        layout.addWidget(self.view_placeholder)

        # 添加一个覆盖的提示 Label (可选，Open3D加载后会被覆盖)
        self.lbl_msg = QLabel("Initializing 3D Engine...", self.view_placeholder)
        self.lbl_msg.setAlignment(Qt.AlignCenter)
        self.lbl_msg.setStyleSheet("color: #666; font-size: 20px; background: transparent;")
        self.lbl_msg.resize(400, 100)  # 随便给个大小

        return container

    def init_open3d_embedding(self):
        """在窗口显示后调用"""
        try:
            # 1. 创建 Open3D 窗口
            self.view3d.ensure_window()

            # 2. 执行嵌入
            # 传入我们的自定义容器 self.view_placeholder
            self.view3d.setup_embedded_window(self.view_placeholder)

            # 3. 隐藏提示文字
            self.lbl_msg.hide()

            self.logger.log("Open3D 引擎已启动并嵌入。", "SUCCESS")
        except Exception as e:
            self.logger.log(f"Open3D 初始化失败: {e}", "ERROR")

    def create_right_panel(self):
        """右侧：算法与数据"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        tabs = QTabWidget()

        # Tab 1: 处理 (降采样/法向)
        tab_proc = QWidget()
        l_proc = QVBoxLayout(tab_proc)

        # 1. 点云采样
        g_pcd = QGroupBox("预处理")
        f_pcd = QFormLayout(g_pcd)
        self.cb_use_pcd = QCheckBox("启用点云模式")
        self.cb_use_pcd.stateChanged.connect(self.slots.on_use_pcd_toggled)

        self.spin_sample_points = QSpinBox()
        self.spin_sample_points.setRange(1000, 5000000)
        self.spin_sample_points.setValue(200000)
        self.spin_sample_points.setSingleStep(50000)

        self.spin_down_voxel = self._create_spin(0.01, 0.0001, 1.0, 0.001, 4)
        self.btn_apply_downsample = QPushButton("应用体素降采样")
        self.btn_apply_downsample.clicked.connect(self.slots.on_apply_downsample)

        f_pcd.addRow(self.cb_use_pcd)
        f_pcd.addRow("采样点数:", self.spin_sample_points)
        f_pcd.addRow("降采样体素:", self.spin_down_voxel)
        l_proc.addWidget(g_pcd)
        l_proc.addWidget(self.btn_apply_downsample)

        # 2. 可视化辅助
        g_vis = QGroupBox("辅助显示")
        f_vis = QFormLayout(g_vis)

        self.cb_show_voxel = QCheckBox("显示体素网格")
        self.cb_show_voxel.stateChanged.connect(self.slots.on_voxel_toggled)
        self.spin_voxel_size = self._create_spin(0.01, 0.001, 1.0)
        self.spin_voxel_size.valueChanged.connect(self.slots.on_voxel_size_changed)

        self.cb_show_normals = QCheckBox("显示法向")
        self.cb_show_normals.stateChanged.connect(self.slots.on_show_normals_toggled)
        self.spin_normal_len = self._create_spin(0.02, 0.001, 1.0)
        self.spin_normal_len.valueChanged.connect(self.slots.on_normal_params_changed)

        # 法向颜色
        self.combo_normal_color = QComboBox()
        self.combo_normal_color.addItems(["红", "绿", "蓝", "白"])
        self.combo_normal_color.currentIndexChanged.connect(self.slots.on_normal_params_changed)

        # 隐藏的 step，使用默认值
        self.spin_normal_step = QSpinBox()
        self.spin_normal_step.setValue(10)
        self.spin_normal_step.setVisible(False)

        f_vis.addRow(self.cb_show_voxel)
        f_vis.addRow("体素大小:", self.spin_voxel_size)
        f_vis.addRow(self.cb_show_normals)
        f_vis.addRow("法向长度:", self.spin_normal_len)
        f_vis.addRow("法向颜色:", self.combo_normal_color)
        l_proc.addWidget(g_vis)
        l_proc.addStretch()

        # Tab 2: NBV
        tab_nbv = QWidget()
        l_nbv = QVBoxLayout(tab_nbv)

        g_nbv = QGroupBox("NBV 规划参数")
        f_nbv = QFormLayout(g_nbv)

        self.combo_nbv_mode = QComboBox()
        self.combo_nbv_mode.addItems(["中心球壳", "表面法向外扩"])
        self.combo_nbv_mode.currentIndexChanged.connect(self.slots.on_nbv_mode_changed)

        self.spin_nbv_voxel = self._create_spin(self.nbv_controller.voxel_size, 0.001, 5.0)
        self.spin_nbv_voxel.valueChanged.connect(self.slots.on_nbv_params_changed)

        self.spin_nbv_candidates = QSpinBox()
        self.spin_nbv_candidates.setRange(1, 2000)
        self.spin_nbv_candidates.setValue(self.nbv_controller.num_candidates)
        self.spin_nbv_candidates.valueChanged.connect(self.slots.on_nbv_params_changed)

        self.spin_nbv_curv_w = self._create_spin(self.nbv_controller.curvature_weight, 0, 10)
        self.spin_nbv_curv_w.valueChanged.connect(self.slots.on_nbv_curvature_weight_changed)

        f_nbv.addRow("模式:", self.combo_nbv_mode)
        f_nbv.addRow("覆盖体素:", self.spin_nbv_voxel)
        f_nbv.addRow("候选数量:", self.spin_nbv_candidates)
        f_nbv.addRow("曲率权重:", self.spin_nbv_curv_w)
        l_nbv.addWidget(g_nbv)

        self.btn_nbv_next = QPushButton("计算下一最佳视点 (NBV)")
        self.btn_nbv_next.setObjectName("primaryButton")
        self.btn_nbv_next.setMinimumHeight(40)
        self.btn_nbv_next.clicked.connect(self.slots.on_nbv_next_clicked)
        l_nbv.addWidget(self.btn_nbv_next)
        l_nbv.addStretch()

        tabs.addTab(tab_proc, "处理")
        tabs.addTab(tab_nbv, "NBV")
        layout.addWidget(tabs)

        # --- C. 列表管理 (底部固定) ---
        grp_list = QGroupBox("扫描帧管理")
        l_list = QVBoxLayout(grp_list)
        l_list.setContentsMargins(5, 5, 5, 5)

        # ========== [新增开始] ==========
        # 添加可视化控制复选框
        l_vis_opts = QHBoxLayout()

        self.cb_show_scan_frustums = QCheckBox("显示视锥")
        # 从 Model 获取默认值 (model_model.py 中已定义)
        self.cb_show_scan_frustums.setChecked(self.model.show_scan_frustums)
        self.cb_show_scan_frustums.toggled.connect(self.slots.on_show_scan_frustums_toggled)

        self.cb_show_scan_coords = QCheckBox("显示坐标")
        self.cb_show_scan_coords.setChecked(self.model.show_scan_coords)
        self.cb_show_scan_coords.toggled.connect(self.slots.on_show_scan_coords_toggled)

        l_vis_opts.addWidget(self.cb_show_scan_frustums)
        l_vis_opts.addWidget(self.cb_show_scan_coords)
        l_vis_opts.addStretch()  # 靠左对齐

        l_list.addLayout(l_vis_opts)
        # ========== [新增结束] ==========

        self.view_table = QTableWidget(0, 3)
        self.view_table.setHorizontalHeaderLabels(["名称", "显示", "颜色"])
        header = self.view_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.view_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.view_table.itemDoubleClicked.connect(self.slots.on_view_table_double_clicked)
        l_list.addWidget(self.view_table)

        # 操作按钮网格
        g_btns = QGridLayout()
        self.btn_show_sel = QPushButton("仅显选中")
        self.btn_show_sel.clicked.connect(self.slots.on_show_only_selected_scan)

        self.btn_del_view = QPushButton("删除")
        self.btn_del_view.setObjectName("dangerButton")
        self.btn_del_view.clicked.connect(self.slots.on_delete_selected_view)

        self.btn_exp_sel = QPushButton("导出选中")
        self.btn_exp_sel.clicked.connect(self.slots.on_export_selected_scan)

        self.btn_exp_all = QPushButton("导出所有")
        self.btn_exp_all.clicked.connect(self.slots.on_export_all_scans)

        self.btn_save_views = QPushButton("保存视点")
        self.btn_save_views.clicked.connect(self.slots.on_save_views_clicked)

        g_btns.addWidget(self.btn_show_sel, 0, 0)
        g_btns.addWidget(self.btn_del_view, 0, 1)
        g_btns.addWidget(self.btn_exp_sel, 1, 0)
        g_btns.addWidget(self.btn_exp_all, 1, 1)
        g_btns.addWidget(self.btn_save_views, 2, 0, 1, 2)

        l_list.addLayout(g_btns)
        layout.addWidget(grp_list)

        scroll.setWidget(container)
        return scroll

    # ================= 辅助方法 =================
    def _create_spin(self, val, min_v, max_v, step=0.1, decimals=2):
        """快速创建统一样式的 SpinBox"""
        sp = QDoubleSpinBox()
        sp.setRange(min_v, max_v)
        sp.setValue(val)
        sp.setSingleStep(step)
        sp.setDecimals(decimals)
        sp.setButtonSymbols(QSpinBox.NoButtons)  # 隐藏上下小箭头，更简洁
        sp.setFixedHeight(22)
        return sp

    def _dual_widget(self, w1, w2):
        """将两个控件并排封装"""
        w = QWidget()
        l = QHBoxLayout(w)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(4)
        l.addWidget(w1)
        l.addWidget(w2)
        return w

    def _tri_widget(self, w1, w2, w3):
        """将三个控件并排封装"""
        w = QWidget()
        l = QHBoxLayout(w)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(4)
        l.addWidget(w1)
        l.addWidget(w2)
        l.addWidget(w3)
        return w

    def closeEvent(self, event):
        self.controller.close()
        event.accept()
