# scan_qt/core/nbv_core.py
import numpy as np
import open3d as o3d

from scan_qt.models.camera_model import CameraModel
from scan_qt.core.camera_core import simulate_scan_raycast


class CoverageGrid:
    """
    用体素网格来记录覆盖情况：
    - bmin, bmax: AABB
    - voxel_size
    - visited: 3D uint8 数组，0=未观测，1=已观测
    """

    def __init__(self, bmin, bmax, voxel_size: float):
        self.bmin = np.array(bmin, dtype=float).copy()
        self.bmax = np.array(bmax, dtype=float).copy()
        self.voxel_size = float(voxel_size)

        extent = np.maximum(self.bmax - self.bmin, 1e-6)
        grid_size = np.ceil(extent / self.voxel_size).astype(int)
        self.grid_size = grid_size  # (nx, ny, nz)

        self.visited = np.zeros(grid_size, dtype=np.uint8)

    def world_to_grid_index(self, pts: np.ndarray):
        """
        pts: (N,3) 世界坐标点
        返回：valid_mask (N,), idx (N,3) int
        """
        pts = np.asarray(pts, dtype=float)
        rel = (pts - self.bmin.reshape(1, 3)) / self.voxel_size
        idx = np.floor(rel).astype(int)  # (N,3)

        valid = (
            (idx[:, 0] >= 0) & (idx[:, 0] < self.grid_size[0]) &
            (idx[:, 1] >= 0) & (idx[:, 1] < self.grid_size[1]) &
            (idx[:, 2] >= 0) & (idx[:, 2] < self.grid_size[2])
        )
        return valid, idx

    def total_voxels(self):
        return int(self.grid_size[0] * self.grid_size[1] * self.grid_size[2])

    def covered_voxels(self):
        return int(np.count_nonzero(self.visited))

    def coverage_ratio(self):
        return self.covered_voxels() / max(1, self.total_voxels())


def build_coverage_grid_from_geom(geom, voxel_size: float) -> CoverageGrid:
    """
    根据几何体（TriangleMesh 或 PointCloud）自动构建覆盖网格。
    自动检查总体素数，如果太大则放大 voxel_size 以控制内存占用。
    """
    if geom is None:
        raise ValueError("geom is None")

    bbox = geom.get_axis_aligned_bounding_box()

    # 注意：min_bound / max_bound 可能返回只读数组，这里显式 copy 一份
    bmin = np.array(bbox.min_bound, dtype=float).copy()
    bmax = np.array(bbox.max_bound, dtype=float).copy()

    # 稍微扩一下边界，避免点在边界上被裁掉
    padding = 0.01 * np.linalg.norm(bmax - bmin)
    if not np.isfinite(padding) or padding <= 0:
        padding = 1e-3

    bmin = bmin - padding
    bmax = bmax + padding

    extent = np.maximum(bmax - bmin, 1e-6)

    # --- 安全控制：限制总体素数 ---
    # 你可以根据机器内存调整这个上限，比如 20~50M
    MAX_VOXELS = 30_000_000  # 3e7，大约 30MB 的 uint8 数组

    # 用户请求的体素
    user_voxel = float(voxel_size)

    # 用用户给的 voxel_size 估算一下体素数量
    grid_size_est = np.ceil(extent / user_voxel).astype(int)
    est_voxels = int(grid_size_est[0] * grid_size_est[1] * grid_size_est[2])

    if est_voxels > MAX_VOXELS:
        # 算一下最小需要的 voxel_size，使得总体素数 <= MAX_VOXELS
        # 简单做法：假设三维各向同性，令 (Lx/v)^3 ≈ MAX_VOXELS
        # 这里取 max_extent，这样保证不超过上限
        max_extent = float(np.max(extent))
        target_res_1d = int(round(MAX_VOXELS ** (1.0 / 3.0)))  # 1D 上的目标格数
        min_voxel = max_extent / max(target_res_1d, 1)

        eff_voxel = max(user_voxel, min_voxel)
        print(
            f"[NBV] Requested voxel_size={user_voxel} leads to "
            f"{est_voxels} voxels, too many. "
            f"Adjusted voxel_size to {eff_voxel:.4f} for coverage grid."
        )
        voxel_size_eff = eff_voxel
    else:
        voxel_size_eff = user_voxel

    return CoverageGrid(bmin, bmax, voxel_size_eff)


def update_coverage_from_scan(coverage: CoverageGrid,
                              scan_pcd: o3d.geometry.PointCloud) -> int:
    """
    把一次扫描点云真正写入 coverage.visited。
    返回：这次扫描带来的“新增覆盖体素数”。
    """
    if coverage is None:
        return 0
    if scan_pcd is None or scan_pcd.is_empty():
        return 0

    pts = np.asarray(scan_pcd.points)
    if pts.shape[0] == 0:
        return 0

    valid, idx = coverage.world_to_grid_index(pts)
    idx = idx[valid]
    if idx.shape[0] == 0:
        return 0

    # 先找出哪些体素原来是0，将变成1
    # 使用 unique 保证每个体素只算一次
    flat_idx = (
        idx[:, 0] * (coverage.grid_size[1] * coverage.grid_size[2]) +
        idx[:, 1] * coverage.grid_size[2] +
        idx[:, 2]
    )
    unique_flat = np.unique(flat_idx)

    z = unique_flat % coverage.grid_size[2]
    y = (unique_flat // coverage.grid_size[2]) % coverage.grid_size[1]
    x = unique_flat // (coverage.grid_size[1] * coverage.grid_size[2])

    before = coverage.visited[x, y, z]
    new_voxels = np.count_nonzero(before == 0)

    coverage.visited[x, y, z] = 1

    return int(new_voxels)


def estimate_coverage_gain_without_commit(coverage: CoverageGrid,
                                          scan_pcd: o3d.geometry.PointCloud) -> int:
    """
    只估计“这次扫描能新增多少体素”，但不写回 coverage.visited。
    用于评估候选视点时的 score。
    """
    if coverage is None:
        return 0
    if scan_pcd is None or scan_pcd.is_empty():
        return 0

    pts = np.asarray(scan_pcd.points)
    if pts.shape[0] == 0:
        return 0

    valid, idx = coverage.world_to_grid_index(pts)
    idx = idx[valid]
    if idx.shape[0] == 0:
        return 0

    flat_idx = (
        idx[:, 0] * (coverage.grid_size[1] * coverage.grid_size[2]) +
        idx[:, 1] * coverage.grid_size[2] +
        idx[:, 2]
    )
    unique_flat = np.unique(flat_idx)

    z = unique_flat % coverage.grid_size[2]
    y = (unique_flat // coverage.grid_size[2]) % coverage.grid_size[1]
    x = unique_flat // (coverage.grid_size[1] * coverage.grid_size[2])

    before = coverage.visited[x, y, z]
    new_voxels = np.count_nonzero(before == 0)
    return int(new_voxels)


def fibonacci_sphere_samples(num_samples: int) -> np.ndarray:
    """
    在单位球面上生成接近均匀分布的点（Fibonacci 采样）。
    返回 (N,3) 数组。
    """
    if num_samples <= 0:
        return np.zeros((0, 3), dtype=float)

    points = []
    offset = 2.0 / num_samples
    increment = np.pi * (3.0 - np.sqrt(5.0))  # 黄金角

    for i in range(num_samples):
        y = ((i * offset) - 1.0) + (offset / 2.0)
        r = np.sqrt(max(0.0, 1.0 - y * y))
        phi = i * increment
        x = np.cos(phi) * r
        z = np.sin(phi) * r
        points.append([x, y, z])

    return np.array(points, dtype=float)


def sample_candidate_views(center: np.ndarray,
                           radius: float,
                           num_candidates: int):
    """
    在以 center 为球心、radius 为半径的球面上采样视点。
    视线方向指向 center。
    返回 list of (pos, dir)。
    """
    center = np.asarray(center, dtype=float)
    if radius <= 0:
        radius = 1.0

    dirs = fibonacci_sphere_samples(num_candidates)
    views = []
    for d in dirs:
        pos = center + radius * d  # 球面上的点
        front = center - pos
        n = np.linalg.norm(front) + 1e-12
        front = front / n
        views.append((pos, front))

    return views


def evaluate_view(mesh: o3d.geometry.TriangleMesh,
                  coverage: CoverageGrid,
                  cam_template: CameraModel,
                  pos: np.ndarray,
                  direction: np.ndarray,
                  num_points: int = 200000):
    """
    对单个候选视点做评估：
      1. 用 template 相机 + 给定 pos/dir 构造一个临时 CameraModel
      2. 用 simulate_scan_raycast 做一次“虚拟扫描”
      3. 用 estimate_coverage_gain_without_commit 估计 coverage 增量
    返回：(gain, virtual_scan_pcd)
    """
    cam = CameraModel()
    # 复制内参
    cam.fov_deg = cam_template.fov_deg
    cam.near = cam_template.near
    cam.far = cam_template.far
    cam.best_distance = cam_template.best_distance
    cam.image_width = cam_template.image_width
    cam.image_height = cam_template.image_height
    cam.up = cam_template.up.copy()

    cam.position = np.asarray(pos, dtype=float)
    cam.direction = np.asarray(direction, dtype=float)

    # 用 raycasting 模拟扫描
    scan_pcd = simulate_scan_raycast(
        cam,
        mesh,
        num_points=num_points,
        scan_color=(1.0, 0.0, 0.0)  # 颜色随意，这里只是虚拟评估
    )

    gain = estimate_coverage_gain_without_commit(coverage, scan_pcd)
    return gain, scan_pcd


def compute_next_best_view(
    mesh: o3d.geometry.TriangleMesh,
    coverage: CoverageGrid,
    cam_template: CameraModel,
    center: np.ndarray,
    radius: float,
    num_candidates: int = 50,
    num_points: int = 200000,
):
    """
    只考虑覆盖率的 NBV：
      - 在球面上采样若干候选视点
      - 对每个视点做虚拟扫描，估计 coverage 增量
      - 选出 gain 最大的视点，返回其信息

    返回：
      None 或 dict:
      {
        "position": (3,),
        "direction": (3,),
        "gain": int,
        "scan_pcd": o3d.geometry.PointCloud,
      }
    """
    if mesh is None or mesh.is_empty():
        print("[NBV] mesh is empty, cannot compute NBV.")
        return None

    center = np.asarray(center, dtype=float)
    if not np.isfinite(center).all():
        print("[NBV] invalid center, cannot compute NBV.")
        return None

    views = sample_candidate_views(center, radius, num_candidates)
    if not views:
        print("[NBV] no candidate views.")
        return None

    best_gain = -1
    best_info = None

    for pos, front in views:
        gain, scan_pcd = evaluate_view(
            mesh, coverage, cam_template,
            pos, front,
            num_points=num_points
        )
        if gain > best_gain:
            best_gain = gain
            best_info = {
                "position": pos,
                "direction": front,
                "gain": gain,
                "scan_pcd": scan_pcd,
            }

    if best_info is None or best_gain <= 0:
        print("[NBV] no candidate gives positive coverage gain.")
        return None

    return best_info
