import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from scipy.special import logsumexp
import os

# 解决 OpenMP 冲突
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"


class HybridMixtureModel:
    def __init__(self, n_clusters=20, max_iter=50, tol=1e-4):
        self.K = n_clusters
        self.max_iter = max_iter
        self.tol = tol
        self.reg_cov = 1e-5

        # 结果存储
        self.labels = None
        self.means = None
        self.covs = None
        self.us = None
        self.kappas = None
        self.weights = None
        self.responsibilities = None

    def _log_vmf_const(self, kappa):
        kappa = np.maximum(kappa, 1e-4)
        log_sinh = np.zeros_like(kappa)
        mask_large = kappa > 50
        mask_small = ~mask_large
        log_sinh[mask_large] = kappa[mask_large] - np.log(2.0)
        log_sinh[mask_small] = np.log(np.sinh(kappa[mask_small]))
        return np.log(kappa) - np.log(4 * np.pi) - log_sinh

    def _estimate_kappa(self, R_bar):
        R_bar = np.clip(R_bar, 0.0, 0.999)
        D = 3
        return (R_bar * (D - R_bar ** 2)) / (1 - R_bar ** 2)

    def fit(self, points, normals):
        """
        核心训练循环：只负责把点云切分成合理的块
        """
        N, D = points.shape

        # --- 1. 强力初始化 ---
        # 计算重心，用于辅助判断方向
        center = np.mean(points, axis=0)

        # 预处理：强制所有法线指向物体中心的外侧 (Outward)
        # 向量 P_center = Point - Center
        # 如果 dot(Normal, P_center) < 0，说明法线指向内部，翻转它
        p_vecs = points - center
        dots = np.sum(normals * p_vecs, axis=1)
        # 翻转指向内部的法线
        flip_mask = dots < 0
        normals[flip_mask] *= -1.0
        print(f"预处理: 翻转了 {np.sum(flip_mask)} 个指向内部的法线。")

        # 联合特征初始化
        bbox_diag = np.linalg.norm(np.max(points, axis=0) - np.min(points, axis=0))
        weight_n = bbox_diag * 0.5  # 强权重
        features = np.hstack([points, normals * weight_n])

        print("初始化: K-Means...")
        kmeans = KMeans(n_clusters=self.K, n_init=10).fit(features)

        self.labels = kmeans.labels_
        self.weights = np.ones(self.K) / self.K
        self.means = kmeans.cluster_centers_[:, :3]  # 只取位置部分
        self.covs = np.array([np.eye(3) * (bbox_diag * 0.02) ** 2 for _ in range(self.K)])

        # 初始方向：直接取聚类内的法线均值
        self.us = np.zeros((self.K, 3))
        for k in range(self.K):
            mask = (self.labels == k)
            if np.sum(mask) > 0:
                avg_n = np.mean(normals[mask], axis=0)
                self.us[k] = avg_n / (np.linalg.norm(avg_n) + 1e-6)
            else:
                self.us[k] = np.array([0, 0, 1])

        self.kappas = np.ones(self.K) * 20.0

        # --- 2. EM 迭代 ---
        # 这里的代码与之前类似，负责优化边界
        # 省略部分重复注释，保留核心逻辑
        print("开始 EM 优化边界...")
        for it in range(self.max_iter):
            # E-Step
            log_resp = np.zeros((N, self.K))
            for k in range(self.K):
                try:
                    diff = points - self.means[k]
                    sigma = self.covs[k] + np.eye(3) * self.reg_cov
                    inv_sigma = np.linalg.inv(sigma)
                    det_sigma = np.linalg.det(sigma)
                    mahal = np.sum(np.dot(diff, inv_sigma) * diff, axis=1)
                    log_gauss = -0.5 * (mahal + np.log(det_sigma + 1e-10) + 3 * np.log(2 * np.pi))
                except:
                    log_gauss = -1000 * np.ones(N)

                log_vmf = self._log_vmf_const(self.kappas[k]) + self.kappas[k] * np.dot(normals, self.us[k])
                log_resp[:, k] = np.log(self.weights[k] + 1e-10) + log_gauss + log_vmf

            log_total = logsumexp(log_resp, axis=1, keepdims=True)
            self.responsibilities = np.exp(log_resp - log_total)

            # M-Step
            Nk = np.sum(self.responsibilities, axis=0) + 1e-10
            self.weights = Nk / N
            self.means = np.dot(self.responsibilities.T, points) / Nk[:, None]

            w_normals = np.dot(self.responsibilities.T, normals)
            r_norm = np.linalg.norm(w_normals, axis=1)
            self.us = w_normals / (r_norm[:, None] + 1e-10)

            r_bar = r_norm / Nk
            self.kappas = np.clip(self._estimate_kappa(r_bar), 0, 500)

            for k in range(self.K):
                diff = points - self.means[k]
                w_diff = diff * np.sqrt(self.responsibilities[:, k][:, None])
                self.covs[k] = np.dot(w_diff.T, w_diff) / Nk[k] + np.eye(3) * self.reg_cov

        self.labels = np.argmax(self.responsibilities, axis=1)
        print("训练完成。")

    def generate_viewpoints_robust(self, points, normals, work_distance):
        """
        【关键修复】:
        不使用训练得到的 self.us (可能存在漂移)，
        而是重新统计聚类内的点云法线，并强制朝向物体外侧。
        """
        viewpoints = []
        global_center = np.mean(points, axis=0)  # 物体整体中心

        for k in range(self.K):
            mask = (self.labels == k)
            if np.sum(mask) == 0: continue

            # 1. 重新计算聚类中心 (Look At Point)
            cluster_center = np.mean(points[mask], axis=0)

            # 2. 重新计算平均法线 (Robust Normal)
            cluster_normals = normals[mask]
            avg_normal = np.mean(cluster_normals, axis=0)
            avg_normal /= (np.linalg.norm(avg_normal) + 1e-6)

            # 3. 强制法线朝外 (Key Logic for Casings)
            # 向量: 中心 -> 聚类中心
            outward_vec = cluster_center - global_center
            if np.dot(avg_normal, outward_vec) < 0:
                # 如果法线指向圆心内部，翻转它
                avg_normal = -avg_normal

            # 4. 计算相机位置
            cam_pos = cluster_center + avg_normal * work_distance

            viewpoints.append({
                "id": k,
                "position": cam_pos,
                "look_at": cluster_center,
                "normal": avg_normal  # 这里的 normal 严格指向相机
            })

        return viewpoints


# ==========================================
# 绘制工具 (保持不变，针对颜色和视锥优化)
# ==========================================
def create_camera_frustum(pos, look_at, up=np.array([0, 0, 1]), size=1.0, color=[0, 1, 0]):
    forward = look_at - pos
    forward /= np.linalg.norm(forward)

    if abs(np.dot(forward, up)) > 0.95: up = np.array([0, 1, 0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, forward)

    # 绘制指向物体的锥体
    # 顶点: 相机位置
    # 底面: 靠近物体的一侧
    h, w, d = size * 0.5, size * 0.6, size

    pts = [pos,
           pos + forward * d - right * w - true_up * h,
           pos + forward * d + right * w - true_up * h,
           pos + forward * d + right * w + true_up * h,
           pos + forward * d - right * w + true_up * h]

    lines = [[0, 1], [0, 2], [0, 3], [0, 4], [1, 2], [2, 3], [3, 4], [4, 1]]
    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(pts),
        lines=o3d.utility.Vector2iVector(lines))
    ls.paint_uniform_color(color)
    return ls


def main():
    # 1. 加载
    ply_path = "#D:/Viewpoint Planning/Auto_Scan/scan_qt/scan_qt/resources/15机匣模型/15-0.66.ply"
    try:
        mesh = o3d.io.read_triangle_mesh(ply_path)
        if len(mesh.vertices) == 0: raise Exception
    except:
        mesh = o3d.geometry.TriangleMesh.create_torus(20, 5)
        mesh.compute_vertex_normals()

    # 2. 预处理 (计算法线)
    pcd = mesh.sample_points_poisson_disk(10000)  # 采样多一点，对机匣更好
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30))
    # 注意：这里先不急着 orient，我们在 fit 里会根据几何中心强制 orient

    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    # 3. 训练
    # 机匣结构复杂，建议 K=30 或更多，确保每个叶片面都有独立视点
    NUM_CLUSTERS = 50
    hmm = HybridMixtureModel(n_clusters=NUM_CLUSTERS, max_iter=50)
    hmm.fit(points, normals)  # fit 内部会处理法线方向

    # 4. 生成视点 (使用 Robust 模式)
    bbox = pcd.get_axis_aligned_bounding_box()
    diag = np.linalg.norm(bbox.get_max_bound() - bbox.get_min_bound())

    WORK_DIST = diag * 0.35  # 相机距离
    CAM_SIZE = WORK_DIST * 0.15  # 显示大小

    # 这里传入的是 fit 内部处理过的法线吗？
    # fit 内部修改的是 copy，所以这里传入原始 normals 没问题，
    # generate_viewpoints_robust 内部会再次做几何一致性检查
    vps = hmm.generate_viewpoints_robust(points, normals, work_distance=WORK_DIST)

    # 5. 可视化
    geoms = []

    # 彩色点云
    import matplotlib.cm as cm
    cmap = cm.get_cmap('tab20')
    colors = np.array([cmap(i % 20)[:3] for i in hmm.labels])
    pcd.colors = o3d.utility.Vector3dVector(colors)
    geoms.append(pcd)

    for i, vp in enumerate(vps):
        c = cmap(i % 20)[:3]

        # 视点连线
        l = o3d.geometry.LineSet()
        l.points = o3d.utility.Vector3dVector([vp['position'], vp['look_at']])
        l.lines = o3d.utility.Vector2iVector([[0, 1]])
        l.colors = o3d.utility.Vector3dVector([c])
        geoms.append(l)

        # 红色箭头: 必须垂直于面片表面
        arrow = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=CAM_SIZE * 0.05, cone_radius=CAM_SIZE * 0.1,
            cylinder_height=CAM_SIZE * 0.6, cone_height=CAM_SIZE * 0.3
        )
        # 旋转箭头到 normal 方向
        z = np.array([0, 0, 1])
        n = vp['normal']
        axis = np.cross(z, n)
        if np.linalg.norm(axis) > 0.01:
            axis = axis / np.linalg.norm(axis)
            angle = np.arccos(np.clip(np.dot(z, n), -1, 1))
            R_mat = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
            arrow.rotate(R_mat, center=[0, 0, 0])
        arrow.translate(vp['look_at'])
        arrow.paint_uniform_color([1, 0, 0])
        geoms.append(arrow)

        # 相机框
        cam = create_camera_frustum(vp['position'], vp['look_at'], size=CAM_SIZE, color=c)
        geoms.append(cam)

    o3d.visualization.draw_geometries(geoms, window_name="Robust Casings VP", width=1200, height=800)


if __name__ == "__main__":
    main()
