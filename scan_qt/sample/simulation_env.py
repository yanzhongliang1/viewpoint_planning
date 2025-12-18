import open3d as o3d
import numpy as np


class VirtualScanner:
    def __init__(self, model_path_or_mesh):
        # 修改：支持传入路径字符串，也支持直接传入 open3d.geometry.TriangleMesh
        if isinstance(model_path_or_mesh, str):
            self.gt_mesh = o3d.io.read_triangle_mesh(model_path_or_mesh)
        else:
            self.gt_mesh = model_path_or_mesh

        self.gt_mesh.compute_vertex_normals()

        # Raycasting 场景
        self.scene = o3d.t.geometry.RaycastingScene()
        # 注意：这里需要确保使用与 Legacy Mesh 相同的变换
        self.gt_mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(self.gt_mesh)
        self.scene.add_triangles(self.gt_mesh_t)

        self.width = 640
        self.height = 480
        self.fov = 60

    def capture(self, position, look_at, up=np.array([0, 0, 1])):
        """
        模拟相机拍摄，返回点云
        """
        # 构建相机外参矩阵 (World to Camera)
        front = look_at - position
        front = front / np.linalg.norm(front)
        right = np.cross(up, front)  # 假设左手系或右手系需根据库调整
        # 防止共线
        if np.linalg.norm(right) < 0.01:
            right = np.array([1, 0, 0])
        right = right / np.linalg.norm(right)
        new_up = np.cross(front, right)

        # 视图矩阵
        R = np.vstack((right, new_up, front))  # 相机坐标系旋转
        T_mat = np.eye(4)
        T_mat[:3, :3] = R.T
        T_mat[:3, 3] = position

        # 逆视图矩阵 (用于 open3d raycasting) - Camera to World
        # Open3D raycasting 实际上需要 eye, center, up 即可生成 rays

        rays = self.scene.create_rays_pinhole(
            fov_deg=self.fov,
            center=look_at,
            eye=position,
            up=new_up,  # 这里使用计算矫正后的up
            width_px=self.width,
            height_px=self.height,
        )

        # 投射光线
        ans = self.scene.cast_rays(rays)

        # 从深度图生成点云
        # 简单处理：提取命中点
        hit = ans['t_hit'].isfinite()
        points = rays[hit][:, :3] + rays[hit][:, 3:] * ans['t_hit'][hit].reshape((-1, 1))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.numpy())

        # 模拟传感器噪声
        pcd = self.apply_noise(pcd)

        return pcd

    def apply_noise(self, pcd):
        points = np.asarray(pcd.points)
        noise = np.random.normal(0, 0.002, points.shape)  # 2mm 噪声
        pcd.points = o3d.utility.Vector3dVector(points + noise)
        return pcd
