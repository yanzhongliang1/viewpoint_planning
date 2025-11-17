# scan_qt/views/model_view.py
import open3d as o3d
import numpy as np

from scan_qt.models.model_model import ModelModel


class ModelView:
    """
    只封装 Open3D 的窗口与渲染：
    - 不直接修改 Model 的数据
    - 只根据 Model 的当前状态进行绘制
    """

    def __init__(self):
        self.vis = None

    # ----------- 窗口管理 -----------

    def ensure_window(self):
        if self.vis is None:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                window_name="Open3D Viewer",
                width=960,
                height=720,
                visible=True
            )

    def close(self):
        if self.vis is not None:
            self.vis.destroy_window()
            self.vis = None

    # ----------- 相机与视图 -----------

    def _reset_camera_from_model(self, model: ModelModel):
        if self.vis is None or model.current_geom is None:
            return

        bbox = model.current_geom.get_axis_aligned_bounding_box()
        model.center = bbox.get_center()
        extent = bbox.get_extent()
        model.radius = float(np.linalg.norm(extent)) + 1e-6

        ctr = self.vis.get_view_control()
        ctr.set_lookat(model.center)
        ctr.set_front([0, 0, -1])
        ctr.set_up([0, -1, 0])
        ctr.set_zoom(0.8)

    def set_camera_pose(self, cam_position, cam_direction, cam_up=None, best_distance=1.0):
        """
        根据给定的相机位姿设置 Open3D 的视图。
        cam_position: (3,) 相机位置
        cam_direction: (3,) 相机朝向（从相机指向物体）
        cam_up: (3,) 上方向（可选，None 时默认 [0,1,0] 去正交）
        best_distance: 用来计算lookat = pos + dir * best_distance
        """
        if self.vis is None:
            return

        pos = np.asarray(cam_position, dtype=float)
        front = np.asarray(cam_direction, dtype=float)
        front = front / (np.linalg.norm(front) + 1e-12)

        if cam_up is None:
            up = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            up = np.asarray(cam_up, dtype=float)

        # 上方向与 front 正交化
        up = up - np.dot(up, front) * front
        up = up / (np.linalg.norm(up) + 1e-12)

        lookat = pos + front * float(best_distance)

        ctr = self.vis.get_view_control()
        ctr.set_lookat(lookat)
        ctr.set_front(front)
        ctr.set_up(up)
        ctr.set_zoom(0.8)

        self._safe_render()


    def set_view_direction(self, model: ModelModel, view_name: str):
        if self.vis is None or model.current_geom is None:
            return

        ctr = self.vis.get_view_control()
        ctr.set_lookat(model.center)

        if view_name == "front":
            front = [0, 0, -1]; up = [0, -1, 0]
        elif view_name == "back":
            front = [0, 0, 1];  up = [0, -1, 0]
        elif view_name == "left":
            front = [-1, 0, 0]; up = [0, -1, 0]
        elif view_name == "right":
            front = [1, 0, 0];  up = [0, -1, 0]
        elif view_name == "top":
            front = [0, -1, 0]; up = [0, 0, -1]
        elif view_name == "bottom":
            front = [0, 1, 0];  up = [0, 0, 1]
        else:
            return

        ctr.set_front(front)
        ctr.set_up(up)
        ctr.set_zoom(0.8)

        self._safe_render()

    # ----------- 场景刷新 -----------

    def render_scene(self, model: ModelModel, recenter=False):
        """
        根据 Model 当前状态重建 Open3D 场景。
        """
        if self.vis is None or model.current_geom is None:
            return

        ctr = self.vis.get_view_control()

        # ===== 关键：如果不想重置视角，就先把当前视角参数保存下来 =====
        saved_params = None
        if not recenter:
            try:
                saved_params = ctr.convert_to_pinhole_camera_parameters()
            except Exception as e:
                print("保存视角参数失败:", e)
                saved_params = None

        # ===== 清空并重新添加几何 =====
        self.vis.clear_geometries()

        # 主几何
        self.vis.add_geometry(model.current_geom)

        # 辅助几何
        if model.show_bbox and model.bbox_geom is not None:
            self.vis.add_geometry(model.bbox_geom)

        if model.show_voxel and model.voxel_grid is not None:
            self.vis.add_geometry(model.voxel_grid)

        if model.show_normals and model.normal_lines is not None:
            self.vis.add_geometry(model.normal_lines)

        # 相机视锥（预览）
        if model.show_camera and model.camera_frustums:
            for frustum in model.camera_frustums:
                self.vis.add_geometry(frustum)

        if model.show_camera and getattr(model, "camera_axes_preview", None) is not None:
            self.vis.add_geometry(model.camera_axes_preview)

        # 扫描点云（全局）
        if model.show_scans and model.scan_clouds:
            for pcd in model.scan_clouds:
                self.vis.add_geometry(pcd)

        # 多视点记录
        for rec in model.view_records:
            if not rec.visible:
                continue
            if rec.frustum is not None:
                self.vis.add_geometry(rec.frustum)
            if rec.axes is not None:
                self.vis.add_geometry(rec.axes)
            if rec.scan_pcd is not None and model.show_scans:
                self.vis.add_geometry(rec.scan_pcd)

        # ===== 相机：要不要重置 lookat？ =====
        if recenter:
            # 第一次加载模型 / 需要重新居中时用
            self._reset_camera_from_model(model)
        else:
            # 不想动视角：把之前保存的参数恢复回去
            if saved_params is not None:
                try:
                    ctr.convert_from_pinhole_camera_parameters(saved_params)
                except Exception as e:
                    print("恢复视角参数失败:", e)

        # 渲染参数
        opt = self.vis.get_render_option()
        opt.point_size = model.point_size

        self._safe_render()

    def _safe_render(self):
        if self.vis is None:
            return
        try:
            self.vis.poll_events()
            self.vis.update_renderer()
        except Exception as e:
            print("渲染异常:", e)

    def update(self):
        """
        提供给 Qt 的定时器直接调用。
        """
        self._safe_render()