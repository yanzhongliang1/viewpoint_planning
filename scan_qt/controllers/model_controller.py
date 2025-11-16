# scan_qt/controllers/nbv_controller.py
import copy
import numpy as np
import open3d as o3d

from scan_qt.models.model_model import ModelModel
from scan_qt.views.model_view import ModelView


class ModelController:
    """
    Controller：负责业务逻辑：
    - 文件加载/保存
    - 各种开关、参数修改
    - 点云采样/降采样/体素/法向构建
    修改 Model，然后调用 View.render_scene() 刷新。
    """

    def __init__(self, model: ModelModel, view: ModelView):
        self.model = model
        self.view = view

    # ----------- 公共：供 UI 调用的一组 API -----------

    def open_ply(self, filename: str) -> bool:
        print("加载文件:", filename)

        self.view.ensure_window()

        # 优先网格
        mesh = o3d.io.read_triangle_mesh(filename)
        if mesh is not None and not mesh.is_empty():
            mesh.compute_vertex_normals()
            self.model.base_geom = mesh
            print("读取为 TriangleMesh")
        else:
            pcd = o3d.io.read_point_cloud(filename)
            if pcd is None or pcd.is_empty():
                print("无法从文件中读取网格或点云")
                return False
            if not pcd.has_normals():
                pcd.estimate_normals()
            self.model.base_geom = pcd
            print("读取为 PointCloud")

        self.model.current_geom = copy.deepcopy(self.model.base_geom)
        self._update_bbox_from_current()

        # 重置视图
        self.view.render_scene(self.model, recenter=True)
        return True

    def save_ply(self, filename: str) -> bool:
        geom = self.model.current_geom
        if geom is None:
            print("当前没有几何体可保存")
            return False

        if isinstance(geom, o3d.geometry.PointCloud):
            ok = o3d.io.write_point_cloud(filename, geom)
        elif isinstance(geom, o3d.geometry.TriangleMesh):
            ok = o3d.io.write_triangle_mesh(filename, geom)
        else:
            print("不支持保存的几何类型:", type(geom))
            return False

        print("保存 PLY 到:", filename, "结果:", ok)
        return bool(ok)

    # ----------- 显示参数/开关 -----------

    def set_point_size(self, size: float):
        self.model.point_size = max(1.0, float(size))
        self.view.render_scene(self.model, recenter=False)

    def toggle_bbox(self, show: bool):
        self.model.show_bbox = bool(show)
        self.view.render_scene(self.model)

    def toggle_use_sampled_pcd(self, use_pcd: bool, num_points: int = 200000):
        if self.model.base_geom is None:
            return
        self.model.use_sampled_pcd = bool(use_pcd)

        if self.model.use_sampled_pcd:
            if self.model.sampled_pcd is None:
                self.model.sampled_pcd = self._ensure_point_cloud_from_base(num_points)
            self.model.current_geom = self.model.sampled_pcd
        else:
            self.model.current_geom = self.model.base_geom

        self._update_bbox_from_current()
        self.view.render_scene(self.model, recenter=True)

    def apply_voxel_downsample(self, voxel_size: float):
        if self.model.base_geom is None:
            return

        voxel_size = max(1e-6, float(voxel_size))

        pcd = self._ensure_point_cloud_from_base()
        if pcd is None:
            return

        down = pcd.voxel_down_sample(voxel_size)
        if not down.has_normals():
            down.estimate_normals()

        self.model.downsampled_pcd = down
        self.model.current_geom = self.model.downsampled_pcd

        self._update_bbox_from_current()
        self.view.render_scene(self.model, recenter=True)

    def toggle_voxel_grid(self, show: bool):
        self.model.show_voxel = bool(show)
        if self.model.show_voxel:
            self._build_voxel_grid()
        self.view.render_scene(self.model)

    def set_voxel_size(self, voxel_size: float):
        self.model.voxel_size = max(1e-6, float(voxel_size))
        if self.model.show_voxel:
            self._build_voxel_grid()
        self.view.render_scene(self.model)

    def toggle_show_normals(self, show: bool):
        self.model.show_normals = bool(show)
        if self.model.show_normals:
            self._build_normal_lines()
        self.view.render_scene(self.model)

    def set_normal_params(self, length=None, step=None, color=None):
        if length is not None:
            self.model.normal_length = max(1e-6, float(length))
        if step is not None:
            self.model.normal_step = max(1, int(step))
        if color is not None:
            self.model.normal_color = np.array(color, dtype=float)

        if self.model.show_normals:
            self._build_normal_lines()
        self.view.render_scene(self.model)

    def set_view(self, view_name: str):
        self.view.set_view_direction(self.model, view_name)

    def update_view(self):
        """
        提供给 Qt 定时器使用。
        """
        self.view.update()

    def close(self):
        self.view.close()

    # ----------- 内部辅助函数：操作 Model 的几何 -----------

    def _update_bbox_from_current(self):
        if self.model.current_geom is None:
            self.model.bbox_geom = None
            return
        bbox = self.model.current_geom.get_axis_aligned_bounding_box()
        bbox.color = (0.0, 1.0, 0.0)
        self.model.bbox_geom = bbox

    def _ensure_point_cloud_from_base(self, num_points: int = 200000):
        base = self.model.base_geom
        if base is None:
            return None

        if isinstance(base, o3d.geometry.PointCloud):
            pcd = copy.deepcopy(base)
            if not pcd.has_normals():
                pcd.estimate_normals()
        elif isinstance(base, o3d.geometry.TriangleMesh):
            pcd = base.sample_points_uniformly(number_of_points=num_points)
            if not pcd.has_normals():
                pcd.estimate_normals()
        else:
            print("不支持的基准几何类型:", type(base))
            return None

        pts = np.asarray(pcd.points)
        if pts.shape[0] > 0:
            colors = np.tile(self.model.default_pcd_color, (pts.shape[0], 1))
            pcd.colors = o3d.utility.Vector3dVector(colors)

        return pcd

    def _build_voxel_grid(self):
        if self.model.current_geom is None:
            self.model.voxel_grid = None
            return

        if isinstance(self.model.current_geom, o3d.geometry.PointCloud):
            pcd = self.model.current_geom
        else:
            pcd = self._ensure_point_cloud_from_base()
            if pcd is None:
                self.model.voxel_grid = None
                return

        vg = o3d.geometry.VoxelGrid.create_from_point_cloud(
            pcd, voxel_size=self.model.voxel_size
        )
        self.model.voxel_grid = vg

    def _build_normal_lines(self):
        if self.model.current_geom is None:
            self.model.normal_lines = None
            return

        if isinstance(self.model.current_geom, o3d.geometry.PointCloud):
            pcd = self.model.current_geom
        else:
            pcd = self._ensure_point_cloud_from_base()
            if pcd is None:
                self.model.normal_lines = None
                return

        if not pcd.has_normals():
            pcd.estimate_normals()

        pts = np.asarray(pcd.points)
        nrm = np.asarray(pcd.normals)
        if pts.shape[0] == 0:
            self.model.normal_lines = None
            return

        idx = np.arange(0, pts.shape[0], self.model.normal_step)
        pts_s = pts[idx]
        nrm_s = nrm[idx]

        line_points = []
        lines = []
        for i, (p, n) in enumerate(zip(pts_s, nrm_s)):
            start = p
            end = p + n * self.model.normal_length
            line_points.append(start)
            line_points.append(end)
            lines.append([2 * i, 2 * i + 1])

        if not line_points:
            self.model.normal_lines = None
            return

        line_points = np.array(line_points, dtype=float)
        lines = np.array(lines, dtype=np.int32)

        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(line_points)
        ls.lines = o3d.utility.Vector2iVector(lines)

        colors = np.tile(self.model.normal_color.reshape(1, 3), (lines.shape[0], 1))
        ls.colors = o3d.utility.Vector3dVector(colors)

        self.model.normal_lines = ls
