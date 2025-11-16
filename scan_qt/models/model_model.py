# scan_qt/models/scene_model.py
import numpy as np


class ModelModel:
    """
    只存几何数据和显示/算法参数，不负责任何 UI 或 Open3D API 调用。
    """

    def __init__(self):
        # 几何对象
        self.base_geom = None          # 原始网格/点云
        self.current_geom = None       # 当前显示几何
        self.sampled_pcd = None        # 网格采样点云
        self.downsampled_pcd = None    # 降采样点云

        # 辅助几何
        self.bbox_geom = None
        self.voxel_grid = None
        self.normal_lines = None
        # ===== 新增：多视点 / 多扫描帧记录 =====
        # 每个视点记录一个对象，里面包含：
        # - name: 视点名称
        # - color: np.array(3,) 颜色
        # - visible: 是否显示
        # - frustum: 视锥 LineSet
        # - axes: 小坐标系 TriangleMesh
        # - scan_pcd: 该视点的扫描点云（可为 None）
        self.view_records = []
        # ===== 新增结束 =====

        # ====== 新增：相机 & 扫描可视化 ======
        # 一个或多个视锥线框
        self.camera_frustums = []  # list[o3d.geometry.LineSet]
        # 多帧扫描点云（每次扫描一帧，加一个点云）
        self.scan_clouds = []  # list[o3d.geometry.PointCloud]
        self.show_camera = True
        self.show_scans = True
        # ====== 以上新增 ======

        # 开关
        self.show_bbox = False
        self.show_voxel = False
        self.show_normals = False
        self.use_sampled_pcd = False

        # 相机
        self.center = np.array([0.0, 0.0, 0.0])
        self.radius = 1.0

        # 点云渲染参数
        self.point_size = 1.0
        self.default_pcd_color = np.array([0.6, 0.6, 0.6])

        # 法向参数
        self.normal_step = 10
        self.normal_length = 0.5
        self.normal_color = np.array([1.0, 0.0, 0.0])

        # 体素参数
        self.voxel_size = 0.1
