import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Any
from scipy.spatial.transform import Rotation as R  # 必须引入这个
from scan_qt.test.robot_comm import Frames


@dataclass
class ViewPointData:
    id: int
    name: str
    local_matrix: np.ndarray
    handle: int = -1


class RobotPath:
    def __init__(self, rc):
        self.rc = rc
        self.sim = rc.sim
        self.viewpoints: List[ViewPointData] = []
        self.viz_container = -1
        self.trail_handle = -1

    def load_viewpoints_from_txt(self, filepath: str):
        """解析TXT，使用 Scipy 构建稳健的局部矩阵，防止 ZMQ 崩溃"""
        self.viewpoints.clear()
        self.clear_visuals()

        print(f"[Path] Loading viewpoints from: {filepath}")

        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 8: continue
                try:
                    vp_id = int(parts[0])
                    name = parts[1]
                    # mm -> m
                    pos = np.array([float(parts[2]), float(parts[3]), float(parts[4])]) / 1000.0
                    n_vec = np.array([float(parts[5]), float(parts[6]), float(parts[7])])

                    # --- 【关键修复】构建无奇异点的旋转矩阵 ---

                    # 1. 归一化法线 (作为 Z 轴)
                    norm = np.linalg.norm(n_vec)
                    if norm < 1e-6:
                        # 防止零向量导致的 NaN
                        z_axis = np.array([0, 0, 1])
                    else:
                        z_axis = n_vec / norm

                    # 2. 动态选择辅助轴 (Up Vector)
                    # 如果 Z 轴太接近世界 Z (0,0,1)，强制改用 Y 轴做辅助，防止 Cross Product 为 0
                    if np.allclose(np.abs(z_axis), [0, 0, 1], atol=1e-2):
                        up = np.array([0, 1, 0])
                    else:
                        up = np.array([0, 0, 1])

                    # 3. 计算 X 和 Y
                    x_axis = np.cross(up, z_axis)
                    x_axis /= np.linalg.norm(x_axis)  # 归一化
                    y_axis = np.cross(z_axis, x_axis)  # 不需要归一化，因为 Z 和 X 正交且单位化

                    # 4. 构建旋转矩阵 (列向量)
                    rot_mat = np.column_stack((x_axis, y_axis, z_axis))

                    # 5. 构建 4x4 变换矩阵
                    T = np.eye(4)
                    T[:3, :3] = rot_mat
                    T[:3, 3] = pos

                    # 6. 安全检查：确保没有 NaN
                    if np.isnan(T).any():
                        print(f"[Path] ⚠️ 警告: ID {vp_id} 计算产生 NaN，已跳过。")
                        continue

                    self.viewpoints.append(ViewPointData(id=vp_id, name=name, local_matrix=T))

                except Exception as e:
                    print(f"[Path] Parse error at ID {parts[0]}: {e}")

    def create_visuals(self):
        if self.viz_container == -1:
            self.viz_container = self.sim.createDummy(0.01)
            self.sim.setObjectAlias(self.viz_container, "VP_Container")

        for vp in self.viewpoints:
            vp.handle = self.sim.createDummy(0.02)
            self.sim.setObjectAlias(vp.handle, f"VP_{vp.name}")
            self.sim.setObjectParent(vp.handle, self.viz_container, True)
            # 初始设为蓝色
            self.sim.setObjectColor(vp.handle, 0, self.sim.colorcomponent_ambient_diffuse, [0, 0, 1])

        self.update_all_dummies_pose()

    def update_all_dummies_pose(self):
        """更新所有 Dummy 的位置，并进行 NaN 检查"""
        # 获取 Receiver 当前世界矩阵
        mat_list = self.sim.getObjectMatrix(self.rc.handles.receiver, self.rc.handles.world)
        T_world_obj = self._sim_matrix_to_numpy(mat_list)

        for vp in self.viewpoints:
            if vp.handle != -1:
                # 矩阵乘法 T_final = T_receiver * T_local
                T_final = T_world_obj @ vp.local_matrix

                # 提取位置
                pos = T_final[:3, 3].tolist()

                # --- 【关键修复】使用 Scipy 提取四元数 ---
                r = R.from_matrix(T_final[:3, :3])
                quat = r.as_quat().tolist()  # [x, y, z, w]

                # 双重保险：检查是否有 NaN
                if any(np.isnan(pos)) or any(np.isnan(quat)):
                    print(f"[Path] Error: NaN detected for VP {vp.id}. Skipping update to prevent crash.")
                    continue

                self.sim.setObjectPose(vp.handle, self.rc.handles.world, pos + quat)

    def get_vp_world_pose(self, index: int):
        if 0 <= index < len(self.viewpoints):
            vp = self.viewpoints[index]
            mat_list = self.sim.getObjectMatrix(self.rc.handles.receiver, self.rc.handles.world)
            T_world_obj = self._sim_matrix_to_numpy(mat_list)
            T_final = T_world_obj @ vp.local_matrix

            pos = T_final[:3, 3].tolist()

            # --- 使用 Scipy 提取四元数 ---
            r = R.from_matrix(T_final[:3, :3])
            quat = r.as_quat().tolist()

            # 检查 NaN
            if any(np.isnan(pos)) or any(np.isnan(quat)):
                print(f"[Path] Error: NaN detected in get_vp_world_pose for VP {vp.id}")
                return None, None

            return pos, quat
        return None, None

    def clear_visuals(self):
        if self.viz_container != -1:
            try:
                self.sim.removeObject(self.viz_container)
            except:
                pass
            self.viz_container = -1

    def init_trail(self):
        if self.trail_handle != -1:
            self.sim.removeDrawingObject(self.trail_handle)
        self.trail_handle = self.sim.addDrawingObject(
            self.sim.drawing_linestrip, 3, 0, -1, 10000, [1, 0, 0]
        )

    def update_trail(self):
        if self.trail_handle != -1:
            pos, _ = self.rc.get_pose(Frames.SCANNER, Frames.WORLD)
            self.sim.addDrawingObjectItem(self.trail_handle, list(pos))

    def clear_trail(self):
        if self.trail_handle != -1:
            try:
                self.sim.removeDrawingObject(self.trail_handle)
            except:
                pass
            self.trail_handle = -1

    def clear_all(self):
        self.clear_trail()
        self.clear_visuals()

    @staticmethod
    def _sim_matrix_to_numpy(m_list):
        m = np.array(m_list).reshape(3, 4)
        return np.vstack([m, [0, 0, 0, 1]])
