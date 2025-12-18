import open3d as o3d
import numpy as np


class VirtualScanner:
    def __init__(self, model_path_or_mesh,
                 min_scan_dist=0.45,  # 传感器物理最小距离
                 max_scan_dist=0.75,  # 传感器物理最大距离
                 max_incidence_angle=75.0):  # 最大入射角阈值(度)

        # 1. 加载模型
        if isinstance(model_path_or_mesh, str):
            self.gt_mesh = o3d.io.read_triangle_mesh(model_path_or_mesh)
        else:
            self.gt_mesh = model_path_or_mesh

        self.gt_mesh.compute_vertex_normals()

        # 2. 设置物理参数
        self.min_scan_dist = min_scan_dist
        self.max_scan_dist = max_scan_dist
        self.max_incidence_angle_rad = np.deg2rad(max_incidence_angle)

        # 3. Raycasting 场景初始化
        self.scene = o3d.t.geometry.RaycastingScene()
        # 转换 mesh 为 Tensor 格式，必须包含法线信息
        self.gt_mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(self.gt_mesh)
        self.scene.add_triangles(self.gt_mesh_t)

        # 相机内参
        self.width = 640
        self.height = 480
        self.fov = 60

    def capture(self, position, look_at, up=np.array([0, 0, 1])):
        """
        模拟符合物理事实的扫描
        """
        # --- A. 构建相机矩阵 (保持不变) ---
        front = look_at - position
        dist = np.linalg.norm(front)
        if dist < 1e-6:
            front = np.array([0, 1, 0])  # 防止重合
        else:
            front /= dist

        right = np.cross(up, front)
        if np.linalg.norm(right) < 0.01: right = np.array([1, 0, 0])
        right /= np.linalg.norm(right)

        new_up = np.cross(front, right)

        # --- B. 光线投射 ---
        rays = self.scene.create_rays_pinhole(
            fov_deg=self.fov,
            center=look_at,
            eye=position,
            up=new_up,
            width_px=self.width,
            height_px=self.height,
        )

        # 执行投射，得到结果字典
        # ans 包含: t_hit (深度), geometry_ids, primitive_ids, primitive_normals 等
        ans = self.scene.cast_rays(rays)

        # --- C. 物理过滤 (核心修改部分) ---

        # 1. 准备数据 (转为 numpy 处理)
        t_hit = ans['t_hit'].numpy()  # 深度图 [H, W]
        # primitive_normals 是击中点的三角形法线 [H, W, 3]
        hit_normals = ans['primitive_normals'].numpy()
        # ray_directions 是光线方向 [H, W, 3]
        ray_dirs = rays.numpy()[:, :, 3:]

        # 2. 基础掩码：是否击中物体
        hit_mask = np.isfinite(t_hit)

        # 3. 物理约束一：距离限制 (Range Filter)
        # 只有在 min_dist 和 max_dist 之间的才算有效
        dist_mask = (t_hit > self.min_scan_dist) & (t_hit < self.max_scan_dist)

        # 4. 物理约束二：入射角限制 (Incidence Angle Filter)
        # 计算光线向量和法线向量的点积
        # 注意：光线是射入，法线是射出，所以通常取反，这里取绝对值即可
        # dot shape: [H, W]
        dot_products = np.abs(np.sum(ray_dirs * hit_normals, axis=2))

        # 阈值：cos(75度) ≈ 0.258
        # 如果点积小于阈值，说明夹角接近90度(垂直)，入射角过大
        angle_threshold = np.cos(self.max_incidence_angle_rad)
        angle_mask = dot_products > angle_threshold

        # 5. 组合所有过滤条件
        valid_mask = hit_mask & dist_mask & angle_mask

        # --- D. 生成最终点云 ---
        if np.sum(valid_mask) == 0:
            return o3d.geometry.PointCloud()

        # 提取有效光线数据
        valid_rays = rays.numpy()[valid_mask]
        valid_t = t_hit[valid_mask]

        # 计算点坐标: Origin + Direction * t
        points = valid_rays[:, :3] + valid_rays[:, 3:] * valid_t.reshape((-1, 1))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 6. 添加传感器噪声 (Distance dependent noise)
        # 距离越远，噪声通常越大
        pcd = self.apply_noise(pcd, valid_t)

        return pcd

    def apply_noise(self, pcd, depths):
        points = np.asarray(pcd.points)
        # 简单的噪声模型：基础噪声 + 距离相关的噪声
        sigma = 0.001 + 0.002 * depths.reshape((-1, 1))  # 1mm base + 0.2% of distance
        noise = np.random.normal(0, sigma, points.shape)
        pcd.points = o3d.utility.Vector3dVector(points + noise)
        return pcd
