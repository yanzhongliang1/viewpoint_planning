# scan_qt/controllers/nbv_controller.py
import numpy as np
import open3d as o3d

from scan_qt.models.model_model import ModelModel
from scan_qt.models.camera_model import CameraModel
from scan_qt.views.model_view import ModelView

from scan_qt.core import nbv_core
from scan_qt.controllers.camera_controller import ViewRecord
from scan_qt.core.camera_core import build_frustum_lines


class NBVController:
    """
    管理 NBV 流程的 Controller：
    - 持有 CoverageGrid
    - 调 nbv_core 计算下一最佳视点
    - 创建 ViewRecord，更新 scene_model + camera_model + view
    """

    def __init__(self,
                 scene_model: ModelModel,
                 view: ModelView,
                 camera_model: CameraModel):
        self.scene_model = scene_model
        self.view = view
        self.camera_model = camera_model

        self.coverage: nbv_core.CoverageGrid | None = None

        # 一些 NBV 参数（先用简单默认值，后面可以接到 UI）
        self.voxel_size = 0.02         # 覆盖网格分辨率
        self.num_candidates = 30       # 每轮评估视点数
        self.weights = dict(
            w_cov=1.0,
            w_dist=0.2,
            w_angle=0.2,
        )

        # 颜色循环（简单复制 camera_controller 的策略）
        self.color_palette = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 1.0, 0.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
        ]
        self.color_index = 0

    # ---------------- 覆盖网格相关 ----------------

    def initialize_coverage(self):
        """
        用当前 base_geom 构造覆盖网格。
        """
        base = self.scene_model.base_geom
        if base is None:
            print("NBV: 没有 base_geom，无法初始化覆盖网格")
            return

        self.coverage = nbv_core.build_initial_coverage_grid(
            base, voxel_size=self.voxel_size
        )
        print("NBV: 初始化 CoverageGrid，size =",
              self.coverage.grid_size, "总体素 =", self.coverage.total_voxels)

    def rebuild_coverage_from_existing_views(self):
        """
        如果你已经手工扫了一些视点，可以用这个函数把已有的 scan_pcd 融入 coverage。
        """
        if self.coverage is None:
            self.initialize_coverage()
            if self.coverage is None:
                return

        for rec in self.scene_model.view_records:
            if rec.scan_pcd is not None and not rec.scan_pcd.is_empty():
                gain = self.coverage.update_with_scan(rec.scan_pcd)
                print(f"NBV: 从已有视点 {rec.name} 融入 coverage，新增 {gain} 体素")

        print("NBV: 目前覆盖:", self.coverage.covered_voxels,
              "/", self.coverage.total_voxels)

    # ---------------- 主流程：跑一步 NBV ----------------

    def run_one_step(self):
        """
        执行一次 NBV 迭代：
          - 从 coverage + 当前几何计算下一最佳视点
          - 把这一帧加入 view_records
          - 更新 CameraModel + 渲染
        """
        base = self.scene_model.base_geom
        if base is None:
            print("NBV: 没有几何，无法执行")
            return

        if self.coverage is None:
            self.initialize_coverage()
            if self.coverage is None:
                return
            # 可选：如果你之前有手动扫过，可以先同步：
            self.rebuild_coverage_from_existing_views()

        center = self.scene_model.center
        # 搜索半径：这里先简单用 camera_model.best_distance
        search_radius = float(self.camera_model.best_distance)

        best = nbv_core.compute_next_best_view(
            base,
            self.coverage,
            self.camera_model,
            center=center,
            search_radius=search_radius,
            num_candidates=self.num_candidates,
            weights=self.weights,
        )

        if best is None:
            print("NBV: 没有找到有效的下一视点（可能已经覆盖完或候选视点无增益）")
            return

        best_pos = best["best_pos"]
        best_dir = best["best_dir"]
        best_score = best["best_score"]
        best_scan = best["best_scan_pcd"]
        gain = best["coverage_gain"]
        dist_pen = best["dist_penalty"]
        ang_pen = best["angle_penalty"]

        # 真正把扫描结果写入 coverage
        real_gain = self.coverage.update_with_scan(best_scan)
        print(
            f"NBV: 选中视点，score={best_score:.4f}, "
            f"覆盖增量(预测/实际)={gain}/{real_gain}, "
            f"dist_pen={dist_pen:.3f}, angle_pen={ang_pen:.3f}"
        )
        print(
            f"NBV: 当前覆盖 {self.coverage.covered_voxels} / {self.coverage.total_voxels}"
        )

        # 更新 CameraModel pose
        self.camera_model.position = best_pos.copy()
        self.camera_model.direction = best_dir.copy()
        # up 保持不变即可

        # 构建视锥和坐标系
        frustum = build_frustum_lines(self.camera_model)

        # 复制 CameraController 里的 _create_camera_axes 的逻辑（简化版）
        front, up, right = self.camera_model.get_normalized_axes()
        pos = self.camera_model.position
        # 按场景尺度/最佳距离生成一个合适的坐标系大小
        scene_scale = float(self.scene_model.radius) if hasattr(self.scene_model, "radius") else 1.0
        size = max(0.05 * scene_scale, 0.1 * self.camera_model.best_distance, 1e-3)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        R = np.column_stack((right, up, front))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos
        axes.transform(T)

        # 颜色策略（循环）
        color = self.color_palette[self.color_index % len(self.color_palette)]
        self.color_index += 1

        view_name = f"NBV_{len(self.scene_model.view_records) + 1}"

        rec = ViewRecord(
            name=view_name,
            color=np.array(color, dtype=float),
            visible=True,
            frustum=frustum,
            axes=axes,
            scan_pcd=best_scan,
            position=best_pos.copy(),
            direction=best_dir.copy(),
        )
        self.scene_model.view_records.append(rec)

        # 打开显示
        self.scene_model.show_camera = True
        self.scene_model.show_scans = True

        # 刷新 Open3D 场景
        self.view.render_scene(self.scene_model, recenter=False)
