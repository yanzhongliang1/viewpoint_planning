# scan_qt/controllers/camera_controller.py
import numpy as np
import open3d as o3d

from scan_qt.models.model_model import ModelModel
from scan_qt.models.camera_model import CameraModel
from scan_qt.views.model_view import ModelView
from scan_qt.core.camera_core import build_frustum_lines, simulate_scan_simple

class ViewRecord:
    def __init__(self, name, color, visible, frustum, axes, scan_pcd, position, direction):
        self.name = name
        self.color = color
        self.visible = visible
        self.frustum = frustum
        self.axes = axes
        self.scan_pcd = scan_pcd
        self.position = position  # np.array(3,)
        self.direction = direction  # np.array(3,)

class CameraController:
    """
    管相机/扫描仪的 Controller：
    - 维护 CameraModel
    - 调 camera_core 构建视锥和扫描点云
    - 把结果写回 ModelModel（camera_frustums / scan_clouds）
    """

    def __init__(self,
                 scene_model: ModelModel,
                 view: ModelView,
                 camera_model: CameraModel):
        self.scene_model = scene_model
        self.view = view
        self.camera_model = camera_model

        # ===== 每帧颜色循环策略 =====
        self.color_palette = [
            (1.0, 0.0, 0.0),  # 红
            (0.0, 1.0, 0.0),  # 绿
            (0.0, 0.0, 1.0),  # 蓝
            (1.0, 1.0, 0.0),  # 黄
            (1.0, 0.0, 1.0),  # 品红
            (0.0, 1.0, 1.0),  # 青
        ]
        self.color_index = 0

    # ===== 参数设置（供 Qt 调） =====
    # 视点坐标系显示
    def _create_camera_axes(self):
        """
        在 camera_model 的位置和朝向上创建一个小坐标系（TriangleMesh）。
        坐标系长度根据模型尺度 / best_distance 自适应。
        """
        front, up, right = self.camera_model.get_normalized_axes()
        pos = self.camera_model.position

        # 根据场景大小和最佳距离，取一个比较合适的尺度
        # 半径太小则给个下限
        scene_scale = float(self.scene_model.radius) if hasattr(self.scene_model, "radius") else 1.0
        size = max(0.05 * scene_scale, 0.1 * self.camera_model.best_distance, 1e-3)

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)

        # 构造旋转矩阵：列向量为坐标轴
        # X 轴 = right, Y 轴 = up, Z 轴 = front
        R = np.column_stack((right, up, front))  # 3x3
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos
        frame.transform(T)

        return frame


    def set_intrinsics(self,
                       fov_deg: float = None,
                       near: float = None,
                       far: float = None,
                       best_distance: float = None,
                       img_w: int = None,
                       img_h: int = None):
        if fov_deg is not None:
            self.camera_model.fov_deg = float(fov_deg)
        if near is not None:
            self.camera_model.near = float(near)
        if far is not None:
            self.camera_model.far = float(far)
        if best_distance is not None:
            self.camera_model.best_distance = float(best_distance)
        if img_w is not None:
            self.camera_model.image_width = int(img_w)
        if img_h is not None:
            self.camera_model.image_height = int(img_h)

    def set_pose(self,
                 pos: np.ndarray = None,
                 direction: np.ndarray = None):
        if pos is not None:
            self.camera_model.position = np.array(pos, dtype=float)
        if direction is not None:
            self.camera_model.direction = np.array(direction, dtype=float)

    def grab_pose_from_current_view(self):
        """
        从当前 Open3D 视图读取视图方向(front / lookat / up)，
        然后利用“已有的 best_distance”在该方向上放置扫描相机：
            pos_scan = lookat - front * best_distance

        注意：不再根据当前视图距离回写 best_distance，
        best_distance 只由用户在 UI 中设置。
        """
        if self.view.vis is None:
            print("Open3D 窗口还未创建，无法读取视图")
            return

        vc = self.view.vis.get_view_control()

        try:
            params = vc.convert_to_pinhole_camera_parameters()
        except Exception as e:
            print("convert_to_pinhole_camera_parameters 失败:", e)
            return

        extr = np.asarray(params.extrinsic, dtype=float)  # 4x4, world -> camera
        R = extr[0:3, 0:3]
        t = extr[0:3, 3]

        # world -> cam: X_cam = R * X_world + t
        # cam 原点在世界坐标中：X_world = -R^T * t
        cam_pos_world = -R.T @ t

        # 有 get_lookat / get_up 的话，用它来定义 front/up，更稳定
        if hasattr(vc, "get_lookat") and hasattr(vc, "get_up"):
            lookat = np.asarray(vc.get_lookat(), dtype=float)
            up = np.asarray(vc.get_up(), dtype=float)
        else:
            # 退化：lookat 用模型中心
            lookat = self.scene_model.center
            up = np.array([0.0, 1.0, 0.0], dtype=float)

        # front 指向“物体方向”
        front = lookat - cam_pos_world
        front = front / (np.linalg.norm(front) + 1e-12)

        # 正交化 up
        up = up - np.dot(up, front) * front
        up = up / (np.linalg.norm(up) + 1e-12)

        # 用“用户设定的 best_distance”来计算扫描相机位置
        d = float(self.camera_model.best_distance)
        scan_pos = lookat - front * d

        # 写回 CameraModel（注意：不改 best_distance）
        self.camera_model.position = scan_pos
        self.camera_model.direction = front
        self.camera_model.up = up

        print("从当前视图更新扫描相机：")
        print("  lookat        =", lookat)
        print("  scan position =", scan_pos)
        print("  direction     =", front)
        print("  best_distance =", self.camera_model.best_distance)

    # ===== 视锥生成 =====

    def update_frustum(self):
        """
        根据当前 camera_model，生成/更新视锥线框。
        现在简单做法：只保留一个最新视锥。
        """
        frustum = build_frustum_lines(self.camera_model)
        self.scene_model.camera_frustums = [frustum]
        self.scene_model.show_camera = True

        # 预览用小坐标系（不是视点记录）
        self.scene_model.camera_axes_preview = self._create_camera_axes()

        # 重新渲染场景
        self.view.render_scene(self.scene_model, recenter=False)

    # ===== 扫描一帧 =====

    def _get_target_point_cloud(self) -> o3d.geometry.PointCloud:
        """
        从当前场景几何得到一个点云，供扫描使用。
        - 如果是点云，直接用
        - 如果是网格，简单均匀采样一下
        """
        base = self.scene_model.base_geom
        if base is None:
            return None

        if isinstance(base, o3d.geometry.PointCloud):
            return base

        if isinstance(base, o3d.geometry.TriangleMesh):
            # 你可以根据需要调大采样点数
            pcd = base.sample_points_uniformly(number_of_points=200000)
            if not pcd.has_normals():
                pcd.estimate_normals()
            return pcd

        print("不支持的基准几何类型用于扫描:", type(base))
        return None

    # 用 raycasting 做遮挡扫描
    def scan_once(self, color=None, use_occlusion: bool = True):
        """
        基于当前 CameraModel + 场景，扫描一帧点云。
        - 如果 base_geom 是 TriangleMesh 且 use_occlusion=True，则用 raycasting 做遮挡检测
        - 否则回退到简单的 FOV+距离裁剪
        """
        base = self.scene_model.base_geom
        if base is None:
            print("没有可扫描的几何")
            return

        # 颜色策略：若未指定，则按palette循环
        if color is None:
            color = self.color_palette[self.color_index % len(self.color_palette)]
            self.color_index += 1

        from scan_qt.core.camera_core import (
            simulate_scan_simple,
            simulate_scan_raycast,
        )

        if isinstance(base, o3d.geometry.TriangleMesh) and use_occlusion:
            scan_pcd = simulate_scan_raycast(self.camera_model, base, scan_color=color)
        else:
            # 回退：先确保有点云
            target_pcd = self._get_target_point_cloud()
            if target_pcd is None:
                print("没有可扫描的点云")
                return
            scan_pcd = simulate_scan_simple(self.camera_model, target_pcd, scan_color=color)

        if scan_pcd is None or scan_pcd.is_empty():
            print("扫描结果为空（视锥内没有可见点）")
            return

        # 为当前视点构造视锥与坐标系
        from scan_qt.core.camera_core import build_frustum_lines
        frustum = build_frustum_lines(self.camera_model)
        axes = self._create_camera_axes()

        # 命名策略：View_1, View_2, ...
        view_name = f"View_{len(self.scene_model.view_records) + 1}"

        rec = ViewRecord(
            name=view_name,
            color=np.array(color, dtype=float),
            visible=True,
            frustum=frustum,
            axes=axes,
            scan_pcd=scan_pcd,
            position=self.camera_model.position.copy(),
            direction=self.camera_model.direction.copy(),
        )
        self.scene_model.view_records.append(rec)

        # 统一开关
        self.scene_model.show_camera = True
        self.scene_model.show_scans = True

        self.view.render_scene(self.scene_model, recenter=False)

    # 给 Qt 视点列表用的管理接口
    def list_views(self):
        """
        返回视点列表信息，给 Qt 填表用。
        """
        result = []
        for i, rec in enumerate(self.scene_model.view_records):
            result.append({
                "index": i,
                "name": rec.name,
                "color": rec.color,
                "visible": rec.visible,
            })
        return result

    def set_view_visibility(self, index: int, visible: bool):
        if 0 <= index < len(self.scene_model.view_records):
            self.scene_model.view_records[index].visible = visible
            self.view.render_scene(self.scene_model, recenter=False)

    def remove_view(self, index: int):
        if 0 <= index < len(self.scene_model.view_records):
            del self.scene_model.view_records[index]
            self.view.render_scene(self.scene_model, recenter=False)

    def go_to_view(self, index: int):
        """
        让 Open3D 相机跳转到某个记录视点，并更新 CameraModel。
        """
        if not (0 <= index < len(self.scene_model.view_records)):
            return

        rec = self.scene_model.view_records[index]

        # 更新 CameraModel
        self.camera_model.position = rec.position.copy()
        self.camera_model.direction = rec.direction.copy()

        # 设置 Open3D 视图
        self.view.set_camera_pose(
            cam_position=self.camera_model.position,
            cam_direction=self.camera_model.direction,
            cam_up=self.camera_model.up,
            best_distance=self.camera_model.best_distance,
        )

    def save_views_to_txt(self, filepath: str):
        """
        将所有视点记录保存为 txt：
        每行：index name px py pz dx dy dz
        """
        lines = []
        for i, rec in enumerate(self.scene_model.view_records):
            p = rec.position
            d = rec.direction
            line = f"{i} {rec.name} {p[0]} {p[1]} {p[2]} {d[0]} {d[1]} {d[2]}"
            lines.append(line)

        with open(filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        print(f"保存视点记录到: {filepath}")

    def export_scan_pcd(self, index: int, filepath: str):
        """
        导出某个视点的扫描点云为 PLY。
        """
        if not (0 <= index < len(self.scene_model.view_records)):
            print("导出失败：索引越界")
            return
        rec = self.scene_model.view_records[index]
        if rec.scan_pcd is None or rec.scan_pcd.is_empty():
            print("该视点没有扫描点云")
            return
        ok = o3d.io.write_point_cloud(filepath, rec.scan_pcd)
        print("导出选中扫描帧:", filepath, "结果:", ok)

    def export_all_scans(self, filepath: str):
        """
        将所有视点的扫描点云合并后导出为一个 PLY。
        """
        pts_list = []
        clr_list = []
        for rec in self.scene_model.view_records:
            if rec.scan_pcd is None or rec.scan_pcd.is_empty():
                continue
            pts = np.asarray(rec.scan_pcd.points)
            pts_list.append(pts)
            if rec.scan_pcd.has_colors():
                clr = np.asarray(rec.scan_pcd.colors)
            else:
                # 若没有颜色，用记录颜色填充
                clr = np.tile(rec.color.reshape(1, 3), (pts.shape[0], 1))
            clr_list.append(clr)

        if not pts_list:
            print("没有可导出的扫描点云")
            return

        all_pts = np.vstack(pts_list)
        all_clr = np.vstack(clr_list)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts)
        pcd.colors = o3d.utility.Vector3dVector(all_clr)

        ok = o3d.io.write_point_cloud(filepath, pcd)
        print("导出所有扫描点云:", filepath, "结果:", ok)
