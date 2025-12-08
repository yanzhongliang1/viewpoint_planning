# scan_qt\viewpoints_views\model_view.py
import open3d as o3d
import numpy as np
import sys, time

# 尝试引入 win32gui 用于窗口嵌入 (仅 Windows)
try:
    import win32gui
    import win32con

    HAS_WIN32 = True
except ImportError:
    HAS_WIN32 = False

from scan_qt.models.model_model import ModelModel


class ModelView:
    """
    负责 Open3D 渲染，并尝试将窗口嵌入到 Qt 界面中。
    """

    def __init__(self):
        self.vis = None
        # 定义窗口名，确保唯一性，防止找到其他窗口
        self.window_name = f"Open3D_Viewer_{id(self)}"
        self.embedded_hwnd = None  # 保存 Open3D 窗口句柄

    # ----------- 窗口管理 -----------

    def ensure_window(self):
        """仅负责创建 Open3D 窗口，不负责嵌入"""
        if self.vis is None:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                window_name=self.window_name,
                width=640, # 初始大小不重要，后面会 resize
                height=480,
                visible=True
            )
            # 设置一些基础渲染选项
            opt = self.vis.get_render_option()
            opt.background_color = np.asarray([0.9, 0.9, 0.9]) # 0.12
            opt.point_size = 2.0

    def setup_embedded_window(self, parent_widget):
        """
        核心方法：将 Open3D 窗口强行塞入 Qt Widget 中
        """
        if not HAS_WIN32 or self.vis is None:
            return

        # 1. 获取 Open3D 窗口句柄
        # 有时窗口创建有延迟，尝试几次
        hwnd = 0
        for _ in range(10):
            hwnd = win32gui.FindWindow(None, self.window_name)
            if hwnd: break
            time.sleep(0.05)

        if not hwnd:
            print("Error: Could not find Open3D window handle.")
            return

        # 2. 获取 Qt 父容器句柄
        parent_hwnd = int(parent_widget.winId())

        # 3. 修改 Open3D 窗口样式：去掉标题栏、边框，设为子窗口
        style = win32gui.GetWindowLong(hwnd, win32con.GWL_STYLE)
        style = style & ~win32con.WS_POPUP  # 去掉弹出式属性
        style = style & ~win32con.WS_CAPTION  # 去掉标题栏 (关键)
        style = style & ~win32con.WS_THICKFRAME  # 去掉调整大小的边框
        style = style & ~win32con.WS_SYSMENU  # 去掉系统菜单
        style = style | win32con.WS_CHILD  # 设为子窗口 (关键)

        win32gui.SetWindowLong(hwnd, win32con.GWL_STYLE, style)

        # 4. 设置父窗口关系
        win32gui.SetParent(hwnd, parent_hwnd)

        # 5. 保存句柄供后续 Resize 使用
        self.embedded_hwnd = hwnd

        # 6. 初始调整大小
        self.resize_to_container(parent_widget)

        print(f"Successfully embedded Open3D window ({hwnd}) into Qt ({parent_hwnd})")

    def resize_to_container(self, parent_widget):
        """
        当 Qt 容器大小改变时，调用此方法强制 Open3D 窗口跟随改变
        """
        if HAS_WIN32 and self.embedded_hwnd:
            try:
                w = parent_widget.width()
                h = parent_widget.height()
                # MoveWindow(hwnd, x, y, width, height, repaint)
                win32gui.MoveWindow(self.embedded_hwnd, 0, 0, w, h, True)
            except Exception as e:
                print(f"Resize failed: {e}")

    def close(self):
        if self.vis is not None:
            self.vis.destroy_window()
            self.vis = None
            self.embedded_hwnd = None

    # ----------- 相机与视图 (保持原有逻辑) -----------
    # ... (_reset_camera_from_model, set_camera_pose 等方法不需要改动，直接复制你原来的) ...

    def _reset_camera_from_model(self, model: ModelModel):
        if self.vis is None or model.current_geom is None: return
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
        if self.vis is None: return
        pos = np.asarray(cam_position, dtype=float)
        front = np.asarray(cam_direction, dtype=float)
        front = front / (np.linalg.norm(front) + 1e-12)
        if cam_up is None:
            up = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            up = np.asarray(cam_up, dtype=float)
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
        if self.vis is None or model.current_geom is None: return
        ctr = self.vis.get_view_control()
        ctr.set_lookat(model.center)
        if view_name == "front":
            front, up = [0, 0, -1], [0, -1, 0]
        elif view_name == "back":
            front, up = [0, 0, 1], [0, -1, 0]
        elif view_name == "left":
            front, up = [-1, 0, 0], [0, -1, 0]
        elif view_name == "right":
            front, up = [1, 0, 0], [0, -1, 0]
        elif view_name == "top":
            front, up = [0, -1, 0], [0, 0, -1]
        elif view_name == "bottom":
            front, up = [0, 1, 0], [0, 0, 1]
        else:
            return
        ctr.set_front(front)
        ctr.set_up(up)
        ctr.set_zoom(0.8)
        self._safe_render()

    def _build_model_axes(self, model: ModelModel):
        if model.current_geom is None:
            model.model_axes = None
            return
        bbox = model.current_geom.get_axis_aligned_bounding_box()
        radius = float(np.linalg.norm(bbox.get_extent())) + 1e-6
        size = 0.15 * radius if radius > 1e-6 else 1.0
        model.model_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)

    # ----------- 场景刷新 -----------
    def render_scene(self, model: ModelModel, recenter=False):
        if self.vis is None: return

        # 窗口 Resize 处理 (Hack)
        if HAS_WIN32 and getattr(self, 'embedded_hwnd', None):
            # 可以在这里添加代码，检测 parent_widget 大小变化并调用 MoveWindow
            # 但为了性能，通常交给 Qt 的 resizeEvent 处理
            pass

        if model.current_geom is None: return

        ctr = self.vis.get_view_control()
        saved_params = None
        if not recenter:
            try:
                saved_params = ctr.convert_to_pinhole_camera_parameters()
            except:
                pass

        self.vis.clear_geometries()
        self.vis.add_geometry(model.current_geom)

        if model.model_axes is None or recenter: self._build_model_axes(model)
        if getattr(model, "show_model_axes", False) and model.model_axes:
            self.vis.add_geometry(model.model_axes)

        if model.show_bbox and model.bbox_geom: self.vis.add_geometry(model.bbox_geom)
        if model.show_voxel and model.voxel_grid: self.vis.add_geometry(model.voxel_grid)
        if model.show_normals and model.normal_lines: self.vis.add_geometry(model.normal_lines)
        if model.show_camera and model.camera_frustums:
            for f in model.camera_frustums: self.vis.add_geometry(f)
        if model.show_camera and getattr(model, "camera_axes_preview", None):
            self.vis.add_geometry(model.camera_axes_preview)
        if model.show_scans and model.scan_clouds:
            for p in model.scan_clouds: self.vis.add_geometry(p)

        for rec in model.view_records:
            if not rec.visible: continue
            if rec.frustum: self.vis.add_geometry(rec.frustum)
            if rec.axes: self.vis.add_geometry(rec.axes)
            if rec.scan_pcd and model.show_scans: self.vis.add_geometry(rec.scan_pcd)

        if recenter:
            self._reset_camera_from_model(model)
        elif saved_params:
            ctr.convert_from_pinhole_camera_parameters(saved_params)

        opt = self.vis.get_render_option()
        opt.point_size = model.point_size
        self._safe_render()

    def _safe_render(self):
        if self.vis is None: return
        try:
            self.vis.poll_events()
            self.vis.update_renderer()
        except:
            pass

    def update(self):
        self._safe_render()
