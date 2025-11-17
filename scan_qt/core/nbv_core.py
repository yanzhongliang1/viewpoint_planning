# scan_qt/core/nbv_core.py
import copy
import numpy as np
import open3d as o3d

from scan_qt.models.camera_model import CameraModel
from scan_qt.core.camera_core import (
    simulate_scan_raycast,
    simulate_scan_simple,
)


class CoverageGrid:
    """
    用规则体素网格描述覆盖率：
    - 空间范围：bmin ~ bmax（来自模型 AABB）
    - 分辨率：voxel_size
    - visited[i,j,k] = 0/1，表示该 voxel 是否已被扫描到
    """

    def __init__(self, bmin, bmax, voxel_size: float):
        self.bmin = np.asarray(bmin, dtype=float)
        self.bmax = np.asarray(bmax, dtype=float)
        self.voxel_size = float(voxel_size)

        extent = self.bmax - self.bmin
        grid_size = np.ceil(extent / self.voxel_size).astype(int)
        grid_size = np.maximum(grid_size, 1)
        self.grid_size = grid_size  # (nx, ny, nz)

        self.visited = np.zeros(
            (grid_size[0], grid_size[1], grid_size[2]), dtype=np.uint8
        )

    @property
    def total_voxels(self) -> int:
        return int(self.visited.size)

    @property
    def covered_voxels(self) -> int:
        return int(self.visited.sum())

    def _points_to_indices(self, pts: np.ndarray):
        """
        把世界坐标点映射到 voxel 索引 (i,j,k)，过滤掉越界的。
        pts: [N,3]
        返回：indices: [M,3] int
        """
        if pts.size == 0:
            return np.zeros((0, 3), dtype=np.int32)

        rel = (pts - self.bmin.reshape(1, 3)) / self.voxel_size
        idx = np.floor(rel).astype(np.int32)  # [N,3]

        # 过滤在 [0, nx/ny/nz) 内的
        valid = (
            (idx[:, 0] >= 0)
            & (idx[:, 1] >= 0)
            & (idx[:, 2] >= 0)
            & (idx[:, 0] < self.grid_size[0])
            & (idx[:, 1] < self.grid_size[1])
            & (idx[:, 2] < self.grid_size[2])
        )
        idx = idx[valid]
        if idx.size == 0:
            return np.zeros((0, 3), dtype=np.int32)

        # 去重，避免同一 voxel 重复
        # 先线性化
        lin = (
            idx[:, 0]
            + idx[:, 1] * self.grid_size[0]
            + idx[:, 2] * self.grid_size[0] * self.grid_size[1]
        )
        lin_unique, _ = np.unique(lin, return_index=True)
        # 反解回 (i,j,k)
        i = lin_unique % self.grid_size[0]
        j = (lin_unique // self.grid_size[0]) % self.grid_size[1]
        k = lin_unique // (self.grid_size[0] * self.grid_size[1])
        idx_unique = np.stack([i, j, k], axis=1).astype(np.int32)
        return idx_unique

    def compute_gain(self, scan_pcd: o3d.geometry.PointCloud) -> int:
        """
        仅计算“如果把这帧扫描融合进来，会增加多少个新覆盖体素”，不修改 visited。
        """
        if scan_pcd is None or scan_pcd.is_empty():
            return 0

        pts = np.asarray(scan_pcd.points)
        idx = self._points_to_indices(pts)
        if idx.size == 0:
            return 0

        # 统计 visited==0 的 voxel 数量
        gain = 0
        for i, j, k in idx:
            if self.visited[i, j, k] == 0:
                gain += 1
        return gain

    def update_with_scan(self, scan_pcd: o3d.geometry.PointCloud) -> int:
        """
        真正把这帧扫描写入 visited，并返回本次新增覆盖的 voxel 数。
        """
        if scan_pcd is None or scan_pcd.is_empty():
            return 0

        pts = np.asarray(scan_pcd.points)
        idx = self._points_to_indices(pts)
        if idx.size == 0:
            return 0

        gain = 0
        for i, j, k in idx:
            if self.visited[i, j, k] == 0:
                self.visited[i, j, k] = 1
                gain += 1
        return gain


def build_initial_coverage_grid(geom, voxel_size: float) -> CoverageGrid:
    """
    根据当前几何（mesh 或 pcd）的 AABB 构造 CoverageGrid。
    """
    bbox = geom.get_axis_aligned_bounding_box()

    # 注意：一定要 copy，避免得到只读数组
    bmin = np.array(bbox.min_bound, dtype=float)  # 或者 np.asarray(...).copy()
    bmax = np.array(bbox.max_bound, dtype=float)

    # 略微扩一点 margin，防止边界数值误差
    margin = 1e-3 * np.linalg.norm(bmax - bmin)
    bmin = bmin - margin
    bmax = bmax + margin

    return CoverageGrid(bmin, bmax, voxel_size)


def sample_candidate_views(center: np.ndarray,
                           radius: float,
                           num_views: int):
    """
    在包围球表面均匀采样 candidate 视点：
    - 位置 pos = center + dir * radius
    - 朝向 front = center - pos（看向模型中心）
    返回 list[(pos, front)]
    """
    center = np.asarray(center, dtype=float).reshape(3)
    radius = float(radius)
    num_views = int(num_views)

    if num_views <= 0:
        return []

    views = []
    # Fibonacci sphere 采样
    phi = (1 + 5 ** 0.5) / 2  # 黄金比例
    for k in range(num_views):
        t = (k + 0.5) / num_views
        y = 1 - 2 * t
        r = np.sqrt(max(0.0, 1 - y * y))
        theta = 2 * np.pi * k / phi
        x = r * np.cos(theta)
        z = r * np.sin(theta)
        dir_vec = np.array([x, y, z], dtype=float)  # 指向外方向
        pos = center + dir_vec * radius
        front = center - pos
        front /= (np.linalg.norm(front) + 1e-12)
        views.append((pos, front))

    return views


def _clone_camera_with_pose(cam: CameraModel,
                            pos: np.ndarray,
                            direction: np.ndarray) -> CameraModel:
    """
    根据模板 CameraModel + 新位置/朝向，构造一个临时 CameraModel。
    不修改原 cam。
    """
    new_cam = CameraModel()
    # 复制内参
    new_cam.fov_deg = cam.fov_deg
    new_cam.near = cam.near
    new_cam.far = cam.far
    new_cam.best_distance = cam.best_distance
    new_cam.image_width = cam.image_width
    new_cam.image_height = cam.image_height

    # 外参
    new_cam.position = np.asarray(pos, dtype=float).reshape(3)
    new_cam.direction = np.asarray(direction, dtype=float).reshape(3)
    new_cam.up = cam.up.copy()
    return new_cam


def _compute_distance_penalty(cam: CameraModel,
                              scan_pcd: o3d.geometry.PointCloud) -> float:
    """
    距离惩罚：偏离最佳采样距离 best_distance 的程度。
    这里用“扫描点云中点到相机的平均距离”来估计。

    dist_penalty = |mean_dist - best_dist| / best_dist （0 越好）
    """
    if scan_pcd is None or scan_pcd.is_empty():
        return 1.0  # 没扫到点，惩罚最大

    pts = np.asarray(scan_pcd.points)
    pos = cam.position.reshape(1, 3)
    dist = np.linalg.norm(pts - pos, axis=1)
    if dist.size == 0:
        return 1.0

    mean_d = float(dist.mean())
    best_d = float(cam.best_distance)
    if best_d <= 1e-9:
        return 0.0

    return abs(mean_d - best_d) / best_d


def _compute_angle_penalty(cam: CameraModel,
                           scan_pcd: o3d.geometry.PointCloud) -> float:
    """
    角度惩罚（成像质量）：
    极简版：用“点相对于光轴的离轴角”做指标，而不是表面法向入射角。
    - 假设视轴 front 上成像最好，越接近 FOV 边缘质量越差。
    - 计算所有扫描点的 angle / (FOV/2) 的平均值，得到 0~1 范围的惩罚。

    angle_penalty = mean( theta_i / (FOV/2) ), clipped 到 [0,1]，越小越好
    """
    if scan_pcd is None or scan_pcd.is_empty():
        return 1.0

    pts = np.asarray(scan_pcd.points)
    pos = cam.position.reshape(1, 3)
    front, _, _ = cam.get_normalized_axes()

    vec = pts - pos  # [N,3]
    dist = np.linalg.norm(vec, axis=1) + 1e-12
    dir_to_pts = vec / dist.reshape(-1, 1)

    cos_theta = np.clip(dir_to_pts @ front, -1.0, 1.0)
    theta = np.arccos(cos_theta)  # [N] 弧度

    fov_rad = np.deg2rad(cam.fov_deg)
    half = max(fov_rad / 2.0, 1e-6)
    norm_angle = theta / half  # 0 ~ 1（理论上）
    norm_angle = np.clip(norm_angle, 0.0, 1.0)

    return float(norm_angle.mean())


def evaluate_candidate_view(
    geom,
    coverage: CoverageGrid,
    cam_template: CameraModel,
    pos: np.ndarray,
    direction: np.ndarray,
    weights=None,
):
    """
    对一个候选视点做“虚拟扫描 + 打分”：
      1. clone camera model + 设置 pose
      2. 利用 raycast/simple 扫描得到 scan_pcd
      3. coverage.compute_gain(scan_pcd) 算 new_coverage
      4. 计算距离惩罚 / 角度惩罚
      5. 按加权公式算 score

    返回：
      (score, scan_pcd, new_coverage, dist_penalty, angle_penalty)
    若扫描结果为空，返回 score=None。
    """
    if weights is None:
        # 可以在外面传入更合适的权重
        weights = dict(w_cov=1.0, w_dist=0.2, w_angle=0.2)

    cam = _clone_camera_with_pose(cam_template, pos, direction)

    # 扫描：mesh 用 raycast，pcd 用简单裁剪
    if isinstance(geom, o3d.geometry.TriangleMesh):
        scan_pcd = simulate_scan_raycast(cam, geom)
    elif isinstance(geom, o3d.geometry.PointCloud):
        scan_pcd = simulate_scan_simple(cam, geom)
    else:
        print("NBV evaluate 不支持的几何类型:", type(geom))
        return None, None, 0, 1.0, 1.0

    if scan_pcd is None or scan_pcd.is_empty():
        return None, scan_pcd, 0, 1.0, 1.0

    # 覆盖增量
    new_cov = coverage.compute_gain(scan_pcd)
    if new_cov <= 0:
        # 没有带来新的覆盖，直接打低分
        dist_pen = _compute_distance_penalty(cam, scan_pcd)
        ang_pen = _compute_angle_penalty(cam, scan_pcd)
        score = -weights["w_dist"] * dist_pen - weights["w_angle"] * ang_pen
        return score, scan_pcd, new_cov, dist_pen, ang_pen

    # 归一化覆盖率（0~1）
    total = max(coverage.total_voxels, 1)
    cov_reward = new_cov / total

    dist_pen = _compute_distance_penalty(cam, scan_pcd)  # 0 越好
    ang_pen = _compute_angle_penalty(cam, scan_pcd)      # 0 越好

    # 极简评分模型：
    #   score = w_cov * cov_reward
    #           - w_dist * dist_pen
    #           - w_angle * ang_pen
    score = (
        weights["w_cov"] * cov_reward
        - weights["w_dist"] * dist_pen
        - weights["w_angle"] * ang_pen
    )

    return float(score), scan_pcd, int(new_cov), float(dist_pen), float(ang_pen)


def compute_next_best_view(
    geom,
    coverage: CoverageGrid,
    cam_template: CameraModel,
    center: np.ndarray,
    search_radius: float,
    num_candidates: int = 30,
    weights=None,
):
    """
    NBV 主流程（单轮）：
      1. 在包围球上采样 num_candidates 个视点
      2. 对每个视点调用 evaluate_candidate_view
      3. 选出 score 最大的视点，返回相关信息

    返回：
      dict:
        {
          "best_pos": ...,
          "best_dir": ...,
          "best_score": float,
          "best_scan_pcd": o3d.geometry.PointCloud,
          "coverage_gain": int,
          "dist_penalty": float,
          "angle_penalty": float,
        }
      若没有合适视点，返回 None
    """
    candidates = sample_candidate_views(center, search_radius, num_candidates)
    if not candidates:
        return None

    best = None
    best_score = -1e9

    for pos, front in candidates:
        score, scan_pcd, gain, dist_pen, ang_pen = evaluate_candidate_view(
            geom,
            coverage,
            cam_template,
            pos,
            front,
            weights=weights,
        )
        if score is None or scan_pcd is None or scan_pcd.is_empty():
            continue
        if gain <= 0:
            # 没有新增覆盖就不考虑
            continue

        if score > best_score:
            best_score = score
            best = dict(
                best_pos=np.asarray(pos, dtype=float),
                best_dir=np.asarray(front, dtype=float),
                best_score=float(score),
                best_scan_pcd=scan_pcd,
                coverage_gain=int(gain),
                dist_penalty=float(dist_pen),
                angle_penalty=float(ang_pen),
            )

    return best
