# scan_qt/visualization/open3d_v.py
import numpy as np
import copy
import open3d as o3d


class Open3DController:
    """
    负责所有和 Open3D 相关的操作：
    - 载入网格或点云
    - 包围盒显示 / 隐藏
    - 点云 / 网格切换（网格均匀采样为点云）
    - 点云降采样
    - 体素网格显示（体素“分割”）
    - 法向可视化
    - 点大小 / 颜色等渲染参数
    """

    def __init__(self):
        # Open3D 可视化窗口
        self.vis = None

        # 原始几何（网格或点云），只在载入时赋值，不直接修改
        self.base_geom = None

        # 当前用于显示的“主几何”（可能是网格，或者采样后的点云，或者降采样结果）
        self.current_geom = None

        # 一些辅助几何
        self.bbox_geom = None        # 包围盒
        self.voxel_grid = None       # 体素网格（体素“分割”）
        self.normal_lines = None     # 法向线段

        # 显示开关
        self.show_bbox = False
        self.show_voxel = False
        self.show_normals = False

        # 当前是否处于“点云模式”（由网格采样得到）
        self.use_sampled_pcd = False
        self.sampled_pcd = None      # 网格采样得到的点云
        self.downsampled_pcd = None  # 降采样后的点云

        # 相机相关
        self.center = np.array([0.0, 0.0, 0.0])
        self.radius = 1.0

        # 点云渲染参数
        self.point_size = 1.0
        self.default_pcd_color = np.array([0.6, 0.6, 0.6])

        # 法向可视化配置
        self.normal_step = 10         # 每隔多少个点画一个法向，避免太密集
        self.normal_length = 0.01     # 法向线长度
        self.normal_color = np.array([1.0, 0.0, 0.0])  # 默认红色

        # 体素化参数
        self.voxel_size = 0.01

    # -------------------- 基础窗口管理 --------------------

    def _ensure_window(self):
        """确保 Open3D 窗口存在"""
        if self.vis is None:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                window_name="Open3D Viewer",
                width=960,
                height=720,
                visible=True
            )

    def close(self):
        """销毁 Open3D 窗口"""
        if self.vis is not None:
            self.vis.destroy_window()
            self.vis = None

    # -------------------- 载入 / 保存 --------------------

    def load_ply(self, filename: str):
        """
        从 PLY 文件读取网格或点云
        优先尝试网格，其次点云
        """
        print("加载文件:", filename)
        self._ensure_window()

        # 尝试网格
        mesh = o3d.io.read_triangle_mesh(filename)
        if mesh is not None and not mesh.is_empty():
            mesh.compute_vertex_normals()
            self.base_geom = mesh
            print("读取为 TriangleMesh")
        else:
            # 尝试点云
            pcd = o3d.io.read_point_cloud(filename)
            if pcd is None or pcd.is_empty():
                print("无法从文件中读取网格或点云")
                return False
            if not pcd.has_normals():
                pcd.estimate_normals()
            self.base_geom = pcd
            print("读取为 PointCloud")

        # 初始化当前显示几何
        self.current_geom = copy.deepcopy(self.base_geom)

        # 根据当前几何更新包围盒、相机等
        self._update_bbox_from_current()
        self._reset_camera()
        self._refresh_scene(recenter=True)

        return True

    def save_ply(self, filename: str) -> bool:
        """
        保存当前几何到 PLY 文件
        如果当前几何是点云 -> write_point_cloud
        如果是网格 -> write_triangle_mesh
        """
        if self.current_geom is None:
            print("当前没有几何体可保存")
            return False

        geom = self.current_geom
        ok = False
        if isinstance(geom, o3d.geometry.PointCloud):
            ok = o3d.io.write_point_cloud(filename, geom)
        elif isinstance(geom, o3d.geometry.TriangleMesh):
            ok = o3d.io.write_triangle_mesh(filename, geom)
        else:
            print("不支持保存的几何类型:", type(geom))
            return False

        print("保存 PLY 到:", filename, "结果:", ok)
        return bool(ok)

    # -------------------- 相机与视图 --------------------

    def _reset_camera(self):
        """根据当前几何的包围盒重置相机"""
        if self.vis is None or self.current_geom is None:
            return

        bbox = self.current_geom.get_axis_aligned_bounding_box()
        self.center = bbox.get_center()
        extent = bbox.get_extent()
        self.radius = float(np.linalg.norm(extent)) + 1e-6

        ctr = self.vis.get_view_control()
        ctr.set_lookat(self.center)
        # 默认从 +Z 方向看向原点，Y 轴向上
        ctr.set_front([0, 0, -1])
        ctr.set_up([0, -1, 0])
        ctr.set_zoom(0.8)

    def set_view(self, view_name: str):
        """
        预留的视点切换（前后左右上下），
        后续你可以把这些逻辑改成由“视角盒正方体”来触发。
        """
        if self.current_geom is None or self.vis is None:
            return

        ctr = self.vis.get_view_control()
        ctr.set_lookat(self.center)

        if view_name == "front":
            front = [0, 0, -1]
            up = [0, -1, 0]
        elif view_name == "back":
            front = [0, 0, 1]
            up = [0, -1, 0]
        elif view_name == "left":
            front = [-1, 0, 0]
            up = [0, -1, 0]
        elif view_name == "right":
            front = [1, 0, 0]
            up = [0, -1, 0]
        elif view_name == "top":
            front = [0, -1, 0]
            up = [0, 0, -1]
        elif view_name == "bottom":
            front = [0, 1, 0]
            up = [0, 0, 1]
        else:
            return

        ctr.set_front(front)
        ctr.set_up(up)
        ctr.set_zoom(0.8)

        self._safe_render()

    # -------------------- 包围盒 --------------------

    def _update_bbox_from_current(self):
        """根据当前几何更新轴对齐包围盒"""
        if self.current_geom is None:
            self.bbox_geom = None
            return
        bbox = self.current_geom.get_axis_aligned_bounding_box()
        bbox.color = (0.0, 1.0, 0.0)  # 绿色包围盒
        self.bbox_geom = bbox

    def toggle_bbox(self, show: bool):
        """开关包围盒显示"""
        self.show_bbox = bool(show)
        self._refresh_scene()

    # -------------------- 点云 / 网格切换与采样 --------------------

    def set_point_size(self, size: float):
        """设置点大小"""
        self.point_size = max(1.0, float(size))
        if self.vis is not None:
            opt = self.vis.get_render_option()
            opt.point_size = self.point_size
            self._safe_render()

    def _ensure_point_cloud_from_base(self, num_points: int = 200000):
        """
        从 base_geom 得到点云：
        - 如果 base_geom 本身是点云 -> 拷贝一份
        - 如果 base_geom 是网格 -> 均匀采样为点云
        """
        if self.base_geom is None:
            return None

        if isinstance(self.base_geom, o3d.geometry.PointCloud):
            pcd = copy.deepcopy(self.base_geom)
            if not pcd.has_normals():
                pcd.estimate_normals()
        elif isinstance(self.base_geom, o3d.geometry.TriangleMesh):
            mesh = self.base_geom
            # 均匀采样点
            pcd = mesh.sample_points_uniformly(number_of_points=num_points)
            if not pcd.has_normals():
                pcd.estimate_normals()
        else:
            print("不支持的基准几何类型:", type(self.base_geom))
            return None

        # 设置默认点云颜色为灰色
        pts = np.asarray(pcd.points)
        if pts.shape[0] > 0:
            colors = np.tile(self.default_pcd_color, (pts.shape[0], 1))
            pcd.colors = o3d.utility.Vector3dVector(colors)

        return pcd

    def toggle_use_sampled_pcd(self, use_pcd: bool, num_points: int = 200000):
        """
        切换“点云模式”：
        - True：对网格进行均匀采样得到点云，并作为 current_geom 显示
        - False：恢复 base_geom
        """
        if self.base_geom is None:
            return

        self.use_sampled_pcd = bool(use_pcd)

        if self.use_sampled_pcd:
            if self.sampled_pcd is None:
                self.sampled_pcd = self._ensure_point_cloud_from_base(num_points=num_points)
            self.current_geom = self.sampled_pcd
        else:
            self.current_geom = self.base_geom

        self._update_bbox_from_current()
        self._refresh_scene(recenter=True)

    # -------------------- 点云降采样 --------------------

    def apply_voxel_downsample(self, voxel_size: float):
        """
        对点云做体素降采样：
        - 以当前几何为点云（或先从 base_geom 转成点云）
        - 得到降采样点云，作为 current_geom 显示
        """
        if self.base_geom is None:
            return

        voxel_size = max(1e-6, float(voxel_size))

        # 先得到点云
        pcd = self._ensure_point_cloud_from_base()
        if pcd is None:
            return

        down = pcd.voxel_down_sample(voxel_size)
        if not down.has_normals():
            down.estimate_normals()

        self.downsampled_pcd = down
        self.current_geom = self.downsampled_pcd

        self._update_bbox_from_current()
        self._refresh_scene(recenter=True)

    # -------------------- 体素“分割”显示 --------------------

    def set_voxel_size(self, voxel_size: float):
        """更新体素大小参数"""
        self.voxel_size = max(1e-6, float(voxel_size))
        if self.show_voxel:
            self._build_voxel_grid()
            self._refresh_scene()

    def toggle_voxel_grid(self, show: bool):
        """
        开关体素网格显示：
        - show == True：基于当前 point cloud 生成 VoxelGrid 并显示
        - show == False：隐藏 VoxelGrid
        """
        self.show_voxel = bool(show)
        if self.show_voxel:
            self._build_voxel_grid()
        self._refresh_scene()

    def _build_voxel_grid(self):
        """根据当前点云几何构建 VoxelGrid"""
        if self.current_geom is None:
            self.voxel_grid = None
            return

        # 需要点云
        if isinstance(self.current_geom, o3d.geometry.PointCloud):
            pcd = self.current_geom
        else:
            # 如果当前是网格，则转成点云
            pcd = self._ensure_point_cloud_from_base()
            if pcd is None:
                self.voxel_grid = None
                return

        vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=self.voxel_size)
        # 体素颜色默认用蓝色，可以再改
        # VoxelGrid 本身没有简单统一设置颜色的接口，这里留默认
        self.voxel_grid = vg

    # -------------------- 法向可视化 --------------------

    def set_normal_params(self, length: float = None, step: int = None, color=None):
        """
        设置法向可视化的参数
        length: 法向线长度
        step: 每隔多少个点绘制一个法向
        color: (r, g, b)
        """
        if length is not None:
            self.normal_length = max(1e-6, float(length))
        if step is not None:
            self.normal_step = max(1, int(step))
        if color is not None:
            self.normal_color = np.array(color, dtype=float)

        if self.show_normals:
            self._build_normal_lines()
            self._refresh_scene()

    def toggle_show_normals(self, show: bool):
        """
        开关法向显示
        注意：Open3D 老版本没有 pcd.clone()，这里用 copy.deepcopy
        """
        self.show_normals = bool(show)
        if self.show_normals:
            self._build_normal_lines()
        self._refresh_scene()

    def _build_normal_lines(self):
        """基于当前几何构建法向线段（LineSet）"""
        if self.current_geom is None:
            self.normal_lines = None
            return

        # 需要点云 + 法向
        if isinstance(self.current_geom, o3d.geometry.PointCloud):
            pcd = self.current_geom
        else:
            # 如果当前是网格，先转换为点云
            pcd = self._ensure_point_cloud_from_base()
            if pcd is None:
                self.normal_lines = None
                return

        if not pcd.has_normals():
            pcd.estimate_normals()

        pts = np.asarray(pcd.points)
        nrm = np.asarray(pcd.normals)
        if pts.shape[0] == 0:
            self.normal_lines = None
            return

        # 采样部分点用于绘制法向
        idx = np.arange(0, pts.shape[0], self.normal_step)
        pts_s = pts[idx]
        nrm_s = nrm[idx]

        # 每个点生成一条线段 [p, p + n * length]
        line_points = []
        lines = []
        for i, (p, n) in enumerate(zip(pts_s, nrm_s)):
            start = p
            end = p + n * self.normal_length
            line_points.append(start)
            line_points.append(end)
            lines.append([2 * i, 2 * i + 1])

        if len(line_points) == 0:
            self.normal_lines = None
            return

        line_points = np.array(line_points, dtype=float)
        lines = np.array(lines, dtype=np.int32)

        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(line_points)
        ls.lines = o3d.utility.Vector2iVector(lines)

        colors = np.tile(self.normal_color.reshape(1, 3), (lines.shape[0], 1))
        ls.colors = o3d.utility.Vector3dVector(colors)

        self.normal_lines = ls

    # -------------------- 场景刷新 --------------------

    def _refresh_scene(self, recenter: bool = False):
        """
        重建场景：
        - 清空所有几何
        - 添加 current_geom + 各种辅助几何（包围盒 / 体素网格 / 法向线）
        """
        if self.vis is None or self.current_geom is None:
            return

        self.vis.clear_geometries()

        # 主几何
        self.vis.add_geometry(self.current_geom)

        # 包围盒
        if self.show_bbox and self.bbox_geom is not None:
            self.vis.add_geometry(self.bbox_geom)

        # 体素网格
        if self.show_voxel and self.voxel_grid is not None:
            self.vis.add_geometry(self.voxel_grid)

        # 法向线
        if self.show_normals and self.normal_lines is not None:
            self.vis.add_geometry(self.normal_lines)

        # 重设相机
        if recenter:
            self._reset_camera()

        # 应用当前点大小等渲染参数
        if self.vis is not None:
            opt = self.vis.get_render_option()
            opt.point_size = self.point_size

        self._safe_render()

    def _safe_render(self):
        """安全刷新渲染，避免因为窗口已销毁导致的 NoneType 错误"""
        if self.vis is None:
            return
        try:
            self.vis.poll_events()
            self.vis.update_renderer()
        except Exception as e:
            # 防止类似 [Open3D WARNING] GLFW Error 导致的崩溃
            print("渲染异常:", e)

    def update(self):
        """
        给 Qt 的定时器调用，
        用于更新 Open3D 窗口事件循环。
        """
        self._safe_render()
