# scan_qt/core/camera_core.py
import numpy as np
import open3d as o3d

from scan_qt.models.camera_model import CameraModel


def build_frustum_lines(cam: CameraModel) -> o3d.geometry.LineSet:
    """
    根据相机参数构造一个视锥的线框（LineSet）。
    坐标：世界坐标系。
    """

    front, up, right = cam.get_normalized_axes()
    pos = cam.position

    fov_rad = np.deg2rad(cam.fov_deg)
    # 视场角这里当成“垂直 FOV”，横向 FOV 用宽高比近似
    aspect = cam.image_width / cam.image_height

    def plane_points(dist):
        # 以相机前方 dist 处为平面中心，构建矩形四个角
        center = pos + front * dist
        half_height = np.tan(fov_rad / 2.0) * dist
        half_width = half_height * aspect

        up_vec = up * half_height
        right_vec = right * half_width

        # 顺序：左上、右上、右下、左下
        p0 = center - right_vec + up_vec
        p1 = center + right_vec + up_vec
        p2 = center + right_vec - up_vec
        p3 = center - right_vec - up_vec
        return [p0, p1, p2, p3]

    # 近远平面四个角
    near_pts = plane_points(cam.near)
    far_pts = plane_points(cam.far)

    points = []
    points.extend(near_pts)
    points.extend(far_pts)
    points = np.array(points, dtype=float)

    # 8 点索引
    # 0-3: near, 4-7: far
    lines = []
    # 近平面
    lines.extend([[0, 1], [1, 2], [2, 3], [3, 0]])
    # 远平面
    lines.extend([[4, 5], [5, 6], [6, 7], [7, 4]])
    # 连接
    lines.extend([[0, 4], [1, 5], [2, 6], [3, 7]])
    lines = np.array(lines, dtype=np.int32)

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines)

    color = np.array([[0.0, 0.0, 1.0]] * len(lines))  # 蓝色视锥
    ls.colors = o3d.utility.Vector3dVector(color)

    return ls


def simulate_scan_simple(cam: CameraModel,
                         target_pcd: o3d.geometry.PointCloud,
                         scan_color=(1.0, 0.0, 0.0)) -> o3d.geometry.PointCloud:
    """
    简单扫描：不考虑遮挡，只根据视场角 + 最近/最远距离裁剪点云。

    target_pcd: 世界坐标下的点云（你可以从网格采样得到）
    返回：新生成一帧扫描点云（带统一颜色）
    """

    if target_pcd is None or target_pcd.is_empty():
        return o3d.geometry.PointCloud()

    pts = np.asarray(target_pcd.points)
    if pts.shape[0] == 0:
        return o3d.geometry.PointCloud()

    front, _, _ = cam.get_normalized_axes()
    pos = cam.position

    # 向量 & 距离
    vec = pts - pos.reshape(1, 3)          # [N, 3]
    dist = np.linalg.norm(vec, axis=1)     # [N]
    # 与前向夹角
    # 防止除零
    dist_safe = dist + 1e-12
    cos_angle = np.einsum('ij,j->i', vec, front) / dist_safe

    fov_rad = np.deg2rad(cam.fov_deg)
    cos_half = np.cos(fov_rad / 2.0)

    # 条件：距离在 [near, far]，且夹角 < FOV/2
    mask = (dist >= cam.near) & (dist <= cam.far) & (cos_angle >= cos_half)

    scan_pts = pts[mask]
    if scan_pts.shape[0] == 0:
        return o3d.geometry.PointCloud()

    scan_pcd = o3d.geometry.PointCloud()
    scan_pcd.points = o3d.utility.Vector3dVector(scan_pts)

    # 颜色
    color_arr = np.tile(np.array(scan_color, dtype=float).reshape(1, 3),
                        (scan_pts.shape[0], 1))
    scan_pcd.colors = o3d.utility.Vector3dVector(color_arr)

    return scan_pcd

def simulate_scan_raycast(cam: CameraModel,
                          mesh: o3d.geometry.TriangleMesh,
                          num_points: int = 200000,
                          scan_color=(1.0, 0.0, 0.0)) -> o3d.geometry.PointCloud:
    """
    使用 Open3D RaycastingScene 对 triangle mesh 做遮挡检测扫描：
    步骤：
      1. 在 mesh 上均匀采样 num_points 个点
      2. 用 FOV + [near, far] 做一次粗裁剪
      3. 对每个候选点发一条从相机位置到该点的 ray，若首次命中距离 == 该点距离，则视为可见
    注意：mesh 必须是 watertight 的效果最好。
    """

    if mesh is None or mesh.is_empty():
        return o3d.geometry.PointCloud()

    # 1. 采样
    pcd = mesh.sample_points_uniformly(number_of_points=num_points)
    pts = np.asarray(pcd.points)
    if pts.shape[0] == 0:
        return o3d.geometry.PointCloud()

    # 2. 先做 FOV + 距离过滤
    front, _, _ = cam.get_normalized_axes()
    pos = cam.position

    vec = pts - pos.reshape(1, 3)
    dist = np.linalg.norm(vec, axis=1)
    dist_safe = dist + 1e-12
    cos_angle = np.einsum("ij,j->i", vec, front) / dist_safe

    fov_rad = np.deg2rad(cam.fov_deg)
    cos_half = np.cos(fov_rad / 2.0)

    mask = (dist >= cam.near) & (dist <= cam.far) & (cos_angle >= cos_half)
    cand_pts = pts[mask]
    cand_vec = vec[mask]
    cand_dist = dist[mask]

    if cand_pts.shape[0] == 0:
        return o3d.geometry.PointCloud()

    # 3. 用 RaycastingScene 做遮挡检测
    # 转成 tensor mesh
    tmesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(tmesh)

    # 构造 rays: (x, y, z, dx, dy, dz)
    dirs = cand_vec / cand_dist.reshape(-1, 1)
    origins = np.tile(pos.reshape(1, 3), (cand_pts.shape[0], 1))
    rays = np.concatenate([origins, dirs], axis=1)  # [N, 6]

    rays_t = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
    ans = scene.cast_rays(rays_t)
    t_hit = ans["t_hit"].numpy()  # [N]

    # t_hit 是沿射线与 mesh 命中的距离，若与 cand_dist 很接近，则说明该点是第一个可见表面
    # 允许一个相对误差 eps
    eps = 1e-3 * np.maximum(cand_dist, 1.0)
    visible_mask = np.isfinite(t_hit) & (np.abs(t_hit - cand_dist) <= eps)

    vis_pts = cand_pts[visible_mask]
    if vis_pts.shape[0] == 0:
        return o3d.geometry.PointCloud()

    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(vis_pts)

    color_arr = np.tile(np.array(scan_color, dtype=float).reshape(1, 3),
                        (vis_pts.shape[0], 1))
    out.colors = o3d.utility.Vector3dVector(color_arr)

    return out