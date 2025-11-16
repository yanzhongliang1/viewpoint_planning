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
