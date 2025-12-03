# scan_qt/controllers/nbv_controller.py
import numpy as np
import open3d as o3d

from scan_qt.models.model_model import ModelModel
from scan_qt.models.camera_model import CameraModel
from scan_qt.viewpoints_views.model_view import ModelView

from scan_qt.core import nbv_core
from scan_qt.core.camera_core import build_frustum_lines
from scan_qt.controllers.camera_controller import ViewRecord


class NBVController:
    """
    负责 NBV 流程：
    - 管理 CoverageGrid（覆盖度）
    - 调 nbv_core 计算下一最佳视点
    - 把结果写入 ModelModel.view_records，并更新 CameraModel
    """

    def __init__(self,
                 scene_model: ModelModel,
                 camera_model: CameraModel,
                 view: ModelView):
        self.scene_model = scene_model
        self.camera_model = camera_model
        self.view = view

        self.coverage: nbv_core.CoverageGrid | None = None

        # 一些 NBV 参数（可以在 UI 里改）
        self.voxel_size = 0.02      # 覆盖网格的体素大小
        self.num_candidates = 50    # 每轮评估的候选视点数量
        self.num_points_per_scan = 200000  # 虚拟扫描采样点数

        # NBV 模式： "sphere" 或 "surface"
        self.mode = "sphere"

        # surface 模式下的曲率相关参数
        self.curvature_knn = 20
        self.curvature_power = 1.0     # 曲率分布权重（抽样时用）
        self.curvature_weight = 1.0    # 曲率对最终 score 的影响权重

    # --------- 内部辅助 ---------

    def _ensure_coverage_initialized(self):
        """
        确保 coverage 已经建立，如果还没有则根据 base_geom 初始化。
        """
        if self.coverage is not None:
            return

        base = self.scene_model.base_geom
        if base is None:
            print("[NBV] no base geometry, cannot build coverage grid.")
            return

        # 如果是点云，也可以直接用；如果是 mesh 更好
        if isinstance(base, o3d.geometry.PointCloud):
            geom_for_bbox = base
        elif isinstance(base, o3d.geometry.TriangleMesh):
            geom_for_bbox = base
        else:
            print("[NBV] unsupported geom type for coverage init:", type(base))
            return

        self.coverage = nbv_core.build_coverage_grid_from_geom(
            geom_for_bbox, self.voxel_size
        )
        print("[NBV] coverage grid initialized with size:",
              self.coverage.grid_size)

        # 初始时，把已有视点的扫描结果也纳入覆盖
        self._update_coverage_with_existing_views()

    def _update_coverage_with_existing_views(self):
        if self.coverage is None:
            return
        total_new = 0
        for rec in self.scene_model.view_records:
            if rec.scan_pcd is not None:
                inc = nbv_core.update_coverage_from_scan(self.coverage,
                                                         rec.scan_pcd)
                total_new += inc
        if total_new > 0:
            print(f"[NBV] coverage updated from existing views, "
                  f"new voxels: {total_new}")

    def _create_camera_axes(self):
        """
        和 CameraController 里一样，在当前 camera_model 位姿上创建小坐标系。
        为了避免循环依赖，这里拷贝一份逻辑。
        """
        front, up, right = self.camera_model.get_normalized_axes()
        pos = self.camera_model.position

        scene_scale = float(self.scene_model.radius) if hasattr(self.scene_model, "radius") else 1.0
        size = max(0.05 * scene_scale, 0.1 * self.camera_model.best_distance, 1e-3)

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)

        R = np.column_stack((right, up, front))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos
        frame.transform(T)

        return frame

    # --------- 对外 API：跑一轮 NBV ---------

    def run_one_step(self):
        """
        进行一次“只考虑覆盖率”的 NBV 迭代：
          - 如果 coverage 还没初始化，则初始化
          - 调用 nbv_core.compute_next_best_view 找下一视点
          - 写回 CameraModel 和 ModelModel.view_records
        """
        self._ensure_coverage_initialized()
        if self.coverage is None:
            return

        base = self.scene_model.base_geom
        if base is None:
            print("[NBV] no base geometry, cannot run NBV.")
            return

        # 目前只考虑 mesh 场景，若是点云你可以根据需要先重建 mesh 或直接用点云版扫描
        if not isinstance(base, o3d.geometry.TriangleMesh):
            print("[NBV] base_geom is not TriangleMesh, NBV now assumes mesh.")
            return

        center = self.scene_model.center
        radius = float(self.camera_model.best_distance)
        if radius <= 0:
            radius = float(self.scene_model.radius)

        # 根据模式选择不同的 NBV 算法
        if self.mode == "sphere":
            result = nbv_core.compute_next_best_view(
                mesh=base,
                coverage=self.coverage,
                cam_template=self.camera_model,
                center=center,
                radius=radius,
                num_candidates=self.num_candidates,
                num_points=self.num_points_per_scan,
            )
        elif self.mode == "surface":
            result = nbv_core.compute_next_best_view_surface(
                mesh=base,
                coverage=self.coverage,
                cam_template=self.camera_model,
                best_distance=self.camera_model.best_distance,
                num_surface_samples=self.num_candidates,
                num_points=self.num_points_per_scan,
                curvature_knn=self.curvature_knn,
                curvature_power=self.curvature_power,
                curvature_weight=self.curvature_weight,
            )
        else:
            print(f"[NBV] unknown mode: {self.mode}")
            return

        if result is None:
            print("[NBV] no NBV found (no positive coverage gain).")
            return

        pos = np.asarray(result["position"], dtype=float)
        direction = np.asarray(result["direction"], dtype=float)
        gain = int(result["gain"])
        scan_pcd = result["scan_pcd"]

        print(f"[NBV] mode={self.mode}, selected NBV with "
              f"gain={gain}, pos={pos}, dir={direction}")
        if "curvature" in result:
            print(f"[NBV] anchor curvature={result['curvature']:.4f}, "
                  f"score={result.get('score', gain):.2f}")

        # 把这次扫描真正写回 coverage（只在非空扫描时）
        if scan_pcd is not None and not scan_pcd.is_empty():
            nbv_core.update_coverage_from_scan(self.coverage, scan_pcd)
            # NEW: 为这次 NBV 扫描生成一个随机颜色
            rand_color = np.random.rand(3)
            # 稍微避免太暗，抬一抬下限
            rand_color = 0.5 + 0.8 * rand_color

            pts_np = np.asarray(scan_pcd.points)
            if pts_np.shape[0] > 0:
                clr = np.tile(rand_color.reshape(1, 3), (pts_np.shape[0], 1))
                scan_pcd.colors = o3d.utility.Vector3dVector(clr)
        else:
            print("[NBV] warning: best NBV scan_pcd is empty, "
                  "coverage not updated.")
            # 空的话就用一个默认颜色
            rand_color = np.array([1.0, 1.0, 0.0], dtype=float)

        # 更新 CameraModel
        self.camera_model.position = pos
        self.camera_model.direction = direction

        # 生成视锥和小坐标系
        frustum = build_frustum_lines(self.camera_model)
        axes = self._create_camera_axes()

        # 视点命名
        view_name = f"NBV_{len(self.scene_model.view_records) + 1}"

        # 用 ViewRecord 记录下来
        # 注意：颜色可以采用某个策略，比如统一用黄色，或者继承 CameraController 的 palette。
        color = rand_color.astype(float)

        rec = ViewRecord(
            name=view_name,
            color=color,
            visible=True,
            frustum=frustum,
            axes=axes,
            scan_pcd=scan_pcd,
            position=pos.copy(),
            direction=direction.copy(),
        )
        self.scene_model.view_records.append(rec)

        # 确保显示扫描结果
        self.scene_model.show_scans = True
        self.scene_model.show_camera = True

        # 重绘
        self.view.render_scene(self.scene_model, recenter=False)

        # 打印当前总体覆盖率
        ratio = self.coverage.coverage_ratio() if self.coverage else 0.0
        print(f"[NBV] current coverage ratio = {ratio:.4f}")
