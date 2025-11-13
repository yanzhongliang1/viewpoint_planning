# scan_qt/core/camera.py
import numpy as np
from dataclasses import dataclass


@dataclass
class CameraParams:
    """相机的基础参数：FOV / 近裁剪面 / 远裁剪面"""
    fov_deg: float = 60.0   # 垂直视场角
    near: float = 0.1
    far: float = 5.0


class Camera:
    """
    Python 版 Camera 核心类，对应 C++ 里:
    pos / direction / up / right + computeUpAndRight
    """

    def __init__(self, params: CameraParams | None = None):
        self.params = params or CameraParams()

        # 对应 C++ 默认构造函数
        self.pos = np.array([0.0, 0.0, 1.0], dtype=float)
        self.direction = np.array([0.0, 0.0, -1.0], dtype=float)
        self.up = np.array([0.0, 1.0, 0.0], dtype=float)
        self.right = np.array([1.0, 0.0, 0.0], dtype=float)

        self._orthonormalize()

    # --------- 内部工具函数：正交化基向量 ---------
    def _orthonormalize(self):
        """类似 C++ Camera::computeUpAndRight()，保证方向基是正交单位正交基"""

        # 规范化 direction
        if np.linalg.norm(self.direction) < 1e-8:
            self.direction = np.array([0.0, 0.0, -1.0], dtype=float)
        self.direction = self.direction / np.linalg.norm(self.direction)

        # 先根据 direction + up 算 right
        r = np.cross(self.direction, self.up)
        if np.linalg.norm(r) < 1e-8:
            # 如果 up 和 direction 共线，选一个新的 up 再算
            if abs(self.direction[1]) < 0.99:
                tmp_up = np.array([0.0, 1.0, 0.0], dtype=float)
            else:
                tmp_up = np.array([1.0, 0.0, 0.0], dtype=float)
            r = np.cross(self.direction, tmp_up)

        self.right = r / np.linalg.norm(r)
        self.up = np.cross(self.right, self.direction)
        self.up = self.up / np.linalg.norm(self.up)

    # --------- 一些常用接口 ---------
    def set_pose(self, pos, front=None, up=None):
        """直接设置相机位置 + 朝向"""
        self.pos = np.asarray(pos, dtype=float)
        if front is not None:
            self.direction = np.asarray(front, dtype=float)
        if up is not None:
            self.up = np.asarray(up, dtype=float)
        self._orthonormalize()

    def look_at(self, target, keep_roll: bool = False):
        """
        让相机看向某个点（类似 gluLookAt 的那种功能简化版）
        target: 世界坐标
        keep_roll: 是否保持当前 roll，不强制使用世界 up
        """
        target = np.asarray(target, dtype=float)
        v = target - self.pos
        if np.linalg.norm(v) < 1e-8:
            return
        self.direction = v / np.linalg.norm(v)

        if not keep_roll:
            self.up = np.array([0.0, 1.0, 0.0], dtype=float)
        self._orthonormalize()

    def get_open3d_view(self, center=None, radius=None):
        """
        转成 open3d view_control 需要的数据：lookat / front / up / zoom
        center 一般用模型中心；radius 目前用不到，可以以后根据半径改 zoom。
        """
        if center is None:
            center = self.pos + self.direction
        if radius is None:
            radius = 1.0

        return dict(
            lookat=np.asarray(center, dtype=float),
            front=self.direction,
            up=self.up,
            zoom=0.8,  # 先给一个固定 zoom，后面你可以根据 radius 做自适应
        )

    # --------- 视锥几何，用于可视化“扫描仪视锥” ---------
    def get_frustum_lines(self, aspect: float = 4.0 / 3.0):
        """
        返回 (points, lines) 用于 open3d.geometry.LineSet 构建视锥线框.

        points: (N, 3) ndarray
        lines:  (M, 2) int 索引
        点索引约定:
          0: 相机中心 C
          1-4: 近裁剪面顶点 (NTL, NTR, NBR, NBL)
          5-8: 远裁剪面顶点 (FTL, FTR, FBR, FBL)
        """
        fov = np.deg2rad(self.params.fov_deg)

        h_near = np.tan(fov / 2.0) * self.params.near
        w_near = h_near * aspect
        h_far = np.tan(fov / 2.0) * self.params.far
        w_far = h_far * aspect

        z = self.direction
        x = self.right
        y = self.up

        nc = self.pos + z * self.params.near  # near center
        fc = self.pos + z * self.params.far   # far center

        # 近裁剪面 4 角
        near_tl = nc + y * h_near - x * w_near
        near_tr = nc + y * h_near + x * w_near
        near_br = nc - y * h_near + x * w_near
        near_bl = nc - y * h_near - x * w_near

        # 远裁剪面 4 角
        far_tl = fc + y * h_far - x * w_far
        far_tr = fc + y * h_far + x * w_far
        far_br = fc - y * h_far + x * w_far
        far_bl = fc - y * h_far - x * w_far

        points = np.vstack([
            self.pos,
            near_tl, near_tr, near_br, near_bl,
            far_tl, far_tr, far_br, far_bl
        ])

        C, NTL, NTR, NBR, NBL, FTL, FTR, FBR, FBL = range(9)
        lines = [
            # 相机中心连到近裁剪面
            [C, NTL], [C, NTR], [C, NBR], [C, NBL],
            # 近裁剪面的框
            [NTL, NTR], [NTR, NBR], [NBR, NBL], [NBL, NTL],
            # 远裁剪面的框
            [FTL, FTR], [FTR, FBR], [FBR, FBL], [FBL, FTL],
            # 近远平面连线
            [NTL, FTL], [NTR, FTR], [NBR, FBR], [NBL, FBL],
        ]
        return points, lines
