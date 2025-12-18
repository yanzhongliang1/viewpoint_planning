import open3d as o3d
import numpy as np
import matplotlib.colors as mcolors


def get_rainbow_color(step, total_steps=30):
    # 修改点：使用了黄金分割比 0.618034 让每一帧的颜色跨度更大，区分度更高
    # s=1.0, v=1.0 保证颜色最亮
    h = (step * 0.618034) % 1.0
    return list(mcolors.hsv_to_rgb([h, 1.0, 1.0]))


def create_camera_actor(position, look_at, scale=0.05):
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=scale * 0.3, resolution=20)
    mesh.paint_uniform_color([0.2, 0.2, 0.2])
    mesh.compute_vertex_normals()

    forward = look_at - position
    norm = np.linalg.norm(forward)
    if norm > 1e-6:
        forward /= norm
        world_up = np.array([0, 0, 1])
        if np.abs(np.dot(forward, world_up)) > 0.99: world_up = np.array([1, 0, 0])
        right = np.cross(world_up, forward)
        right /= np.linalg.norm(right)
        up = np.cross(forward, right)

        R = np.vstack((right, up, forward)).T
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = position
        mesh.transform(T)
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale * 1.5)
        axis.transform(T)
        return mesh, axis
    return mesh, o3d.geometry.TriangleMesh()


def create_trajectory_lines(history_points):
    if len(history_points) < 2: return o3d.geometry.LineSet()
    points = o3d.utility.Vector3dVector(history_points)
    lines = [[i, i + 1] for i in range(len(history_points) - 1)]
    colors = [[0, 0, 0] for _ in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = points
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def create_heatmap_cloud(candidates):
    if not candidates: return o3d.geometry.PointCloud()
    positions = [c['position'] for c in candidates]
    scores = [c['score'] for c in candidates]

    valid_scores = [s for s in scores if s > -500]
    if not valid_scores: return o3d.geometry.PointCloud()

    min_s, max_s = min(valid_scores), max(valid_scores)
    colors = []
    for s in scores:
        if s < -500:
            colors.append([0.8, 0.8, 0.8])
        else:
            n = (s - min_s) / (max_s - min_s + 1e-6)
            colors.append([n, 0, 1 - n])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def get_progress_bar(percentage, length=30):
    fill = int(length * percentage / 100)
    bar = '█' * fill + '-' * (length - fill)
    return f"|{bar}| {percentage:.1f}%"
