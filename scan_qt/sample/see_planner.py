import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


class VoxelGrid:
    """
    简易概率体素地图 (Probabilistic Voxel Map)
    用于严格区分: Known-Free, Known-Occupied, Unknown
    """

    def __init__(self, resolution=0.05, bounds=((-1, 1), (-1, 1), (-1, 1))):
        self.resolution = resolution
        self.origin = np.array([b[0] for b in bounds])
        self.dims = np.ceil((np.array([b[1] for b in bounds]) - self.origin) / resolution).astype(int)

        # 状态: 0=Unknown, 1=Free, 2=Occupied
        self.grid = np.zeros(self.dims, dtype=np.uint8)
        self.visited_voxels = set()  # 记录被光线穿过的体素

    def world_to_grid(self, points):
        idx = ((points - self.origin) / self.resolution).astype(int)
        # Clip to bounds
        for i in range(3):
            idx[:, i] = np.clip(idx[:, i], 0, self.dims[i] - 1)
        return idx

    def mark_occupied(self, points):
        """标记点云所在位置为 Occupied"""
        if len(points) == 0: return
        idx = self.world_to_grid(points)
        self.grid[idx[:, 0], idx[:, 1], idx[:, 2]] = 2  # Occupied

    def mark_free_raycast(self, origin, hits):
        """
        简化版光线投射：标记相机到击中点之间的区域为 Free
        为了性能，我们只采样光线上的几个关键点，而不是 Bresenham 算法
        """
        if len(hits) == 0: return

        # 向量化计算
        # 对每个击中点，在 Origin 和 Hit 之间采样 5 个点，标记为 Free
        steps = np.linspace(0, 1, 6)[1:-1]  # 排除起点和终点
        for s in steps:
            interp_points = origin + (hits - origin) * s
            idx = self.world_to_grid(interp_points)
            # 只有当该体素原来是 Unknown (0) 时才标记为 Free (1)
            # 不要覆盖 Occupied (2)
            mask = self.grid[idx[:, 0], idx[:, 1], idx[:, 2]] == 0
            if np.any(mask):
                valid_idx = idx[mask]
                self.grid[valid_idx[:, 0], valid_idx[:, 1], valid_idx[:, 2]] = 1

    def count_unknown_voxels_in_view(self, cam_pos, look_at, fov_deg=60, max_dist=1.0):
        """
        【核心】计算信息增益：视锥体内的 Unknown 体素数量
        """
        direction = look_at - cam_pos
        dist = np.linalg.norm(direction)
        if dist < 1e-4: return 0
        direction /= dist

        # 简化计算：在视锥体内随机采样点，检查是否 Unknown
        # 构建一个视锥体采样云
        num_samples = 200
        # 锥体底面半径
        r = max_dist * np.tan(np.deg2rad(fov_deg / 2))

        # 随机生成锥体内向量
        dists = np.random.uniform(0.2, max_dist, num_samples)
        offsets = np.random.uniform(-r, r, (num_samples, 3))
        # 简单的视锥模拟：沿主轴分布
        sample_pts = cam_pos + direction * dists[:, np.newaxis] + offsets * (dists[:, np.newaxis] / max_dist) * 0.5

        idx = self.world_to_grid(sample_pts)
        values = self.grid[idx[:, 0], idx[:, 1], idx[:, 2]]

        # 统计 Unknown (0) 的数量
        return np.sum(values == 0)


class SEEPlanner:
    def __init__(self, sensor_params):
        # 传感器参数
        self.min_dist = sensor_params['min_dist']
        self.max_dist = sensor_params['max_dist']
        # 计算最佳观测距离 (Sweet Spot)
        self.optimal_dist = (self.min_dist + self.max_dist) / 2.0
        self.fov = sensor_params.get('fov', 60)

        # 地图
        self.voxel_map = VoxelGrid(resolution=0.04, bounds=((-0.8, 0.8), (-0.8, 0.8), (-0.8, 0.8)))
        self.pcd = o3d.geometry.PointCloud()
        self.kdtree = None

        self.history = []

    def _generate_sphere_points(self, n):
        """Fibonacci Sphere for uniform sampling"""
        indices = np.arange(0, n, dtype=float) + 0.5
        phi = np.arccos(1 - 2 * indices / n)
        theta = np.pi * (1 + 5 ** 0.5) * indices
        x, y, z = np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)
        return np.column_stack([x, y, z])

    def update_map(self, new_pcd, cam_pos):
        # 1. 融合点云
        self.pcd += new_pcd
        # 必须降采样，否则内存和计算量都会失控
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.02)

        # 2. 更新体素地图 (Occupied)
        points = np.asarray(self.pcd.points)
        if len(points) > 0:
            self.voxel_map.mark_occupied(points)
            # 只有有点的时候才更新 KDTree
            self.kdtree = cKDTree(points)

        # 3. 更新体素地图 (Free)
        if len(new_pcd.points) > 0:
            hits = np.asarray(new_pcd.voxel_down_sample(0.05).points)
            self.voxel_map.mark_free_raycast(cam_pos, hits)

    def _generate_candidates(self, current_pos):
        """
        【修改版】基于当前点云表面的法线方向生成候选视点
        """
        candidates = []

        # 如果点云为空（第一帧），我们需要一个初始视点，或者生成一个默认的包围球
        if len(self.pcd.points) < 50:
            return self._generate_initial_sphere()

        # 1. 提取种子点：为了效率，我们对当前累积的点云做更稀疏的降采样
        # 比如每 10cm 一个种子点
        seed_pcd = self.pcd.voxel_down_sample(voxel_size=0.08)

        # 2. 估计法线
        seed_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))

        points = np.asarray(seed_pcd.points)
        normals = np.asarray(seed_pcd.normals)

        # 物体中心估算（假设归一化后在原点，或用当前点云中心）
        object_center = np.array([0, 0, 0])

        for i in range(len(points)):
            pt = points[i]
            n = normals[i]

            # 3. 矫正法线方向：法线必须指向物体外侧
            # 简单的启发式：法线应该大致指向远离中心的方向
            # 向量 (pt - center) 与 n 的点积应该大于 0
            if np.dot(n, (pt - object_center)) < 0:
                n = -n

            # 4. 生成候选位置
            # Position = 表面点 + 法线 * 最佳距离
            # 这样相机就会正对着表面点
            candidate_pos = pt + n * self.optimal_dist

            # 5. 生成 Look At
            # 相机看向表面点
            look_at = pt

            # 6. 简单的边界检查（防止生成的点在地板下或者太远）
            if candidate_pos[2] < -0.5: continue  # 假设地面在 -0.5 以下

            candidates.append({
                'position': candidate_pos,
                'look_at': look_at,
                'type': 'surface_normal',
                'raw_gain': 0,
                'cost': 0,
                'score': -np.inf
            })

        # 为了防止陷入局部最优（只盯着看过的地方看），我们通常会
        # 混合少量随机的“全局球”采样，或者针对边界点采样。
        # 这里为了演示“法线采样”的效果，我保留少量全局采样作为补充。
        global_candidates = self._generate_initial_sphere(num_points=30)  # 稀疏的全局球
        candidates.extend(global_candidates)

        return candidates

    def _generate_initial_sphere(self, num_points=100):
        """生成围绕原点的球形采样（用于初始化或补充）"""
        candidates = []
        indices = np.arange(0, num_points, dtype=float) + 0.5
        phi = np.arccos(1 - 2 * indices / num_points)
        theta = np.pi * (1 + 5 ** 0.5) * indices
        x, y, z = np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)
        sphere_points = np.column_stack([x, y, z])

        target_dist = self.optimal_dist
        center = np.array([0, 0, 0])

        for dir_vec in sphere_points:
            pos = center + dir_vec * target_dist
            candidates.append({
                'position': pos,
                'look_at': center,
                'type': 'global',
                'raw_gain': 0,
                'cost': 0,
                'score': -np.inf
            })
        return candidates

    def evaluate_view(self, view, current_pos):
        # 1. 安全性检查 (碰撞检测)
        # 查询视点位置最近的障碍物
        dist, _ = self.kdtree.query(view['position'], k=1)
        if dist < 0.15:  # 安全距离
            return -np.inf  # 碰撞

        # 2. 信息增益 (Information Gain)
        # 计算视锥内有多少 Unknown 体素
        ig = self.voxel_map.count_unknown_voxels_in_view(
            view['position'], view['look_at'], self.fov, self.max_dist
        )
        if ig == 0:
            return -100.0  # 无增益，但比碰撞好

        # 3. 移动代价 (Movement Cost)
        dist_move = np.linalg.norm(view['position'] - current_pos)

        # 4. 历史惩罚 (Repetition Penalty) - 强约束
        # 如果距离任何一个历史点太近，分数大幅下降
        min_hist_dist = np.inf
        for h_pos in self.history:
            d = np.linalg.norm(view['position'] - h_pos)
            if d < min_hist_dist: min_hist_dist = d

        penalty = 0
        if min_hist_dist < 0.25:  # 强排斥半径
            penalty = 5000.0
        elif min_hist_dist < 0.5:
            penalty = 100.0 * (0.5 - min_hist_dist)

        # 综合评分公式 (Utility Function)
        # Score = Gain * weight - Cost - Penalty
        score = ig * 1.0 - dist_move * 5.0 - penalty

        view['raw_gain'] = ig
        view['cost'] = dist_move
        return score

    def get_next_view(self, current_pos):
        if len(self.pcd.points) == 0:
            # 初始状态，随便给个视角
            return {'position': np.array([0.5, 0, 0.5]), 'look_at': np.array([0, 0, 0])}, []

        candidates = self._generate_candidates(current_pos)

        best_view = None
        best_score = -np.inf

        processed_candidates = []

        # 并行计算或循环计算评分
        for cand in candidates:
            score = self.evaluate_view(cand, current_pos)
            cand['score'] = score
            processed_candidates.append(cand)

            if score > best_score:
                best_score = score
                best_view = cand

        # 记录历史
        if best_view:
            self.history.append(best_view['position'])
            # 限制历史长度，防止越跑越慢，但要保留足够的记忆
            if len(self.history) > 50:
                self.history.pop(0)

        return best_view, processed_candidates
