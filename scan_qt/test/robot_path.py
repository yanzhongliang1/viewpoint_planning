import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Any
from scan_qt.test.robot_comm import Frames


@dataclass
class ViewPointData:
    id: int
    name: str
    # 核心数据：相对于 Receiver 的局部变换矩阵 (4x4)
    local_matrix: np.ndarray
    # 句柄：对应场景中的 Dummy
    handle: int = -1


class RobotPath:
    def __init__(self, rc):
        self.rc = rc
        self.sim = rc.sim
        self.viewpoints: List[ViewPointData] = []

        # 视觉容器
        self.viz_container = -1
        self.trail_handle = -1

    def load_viewpoints_from_txt(self, filepath: str):
        """解析TXT，构建局部矩阵"""
        self.viewpoints.clear()
        # 清理旧的 dummy
        self.clear_visuals()

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

                    # 构建局部矩阵 T_obj_vp
                    R = self._calc_rotation_matrix_from_dir(n_vec)
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = pos

                    self.viewpoints.append(ViewPointData(id=vp_id, name=name, local_matrix=T))
                except Exception as e:
                    print(f"[Path] Parse error: {e}")

    def create_visuals(self):
        """基于当前 Receiver 的位置，初始化 Dummy"""
        if self.viz_container == -1:
            self.viz_container = self.sim.createDummy(0.01)
            self.sim.setObjectAlias(self.viz_container, "VP_Container")

        for vp in self.viewpoints:
            vp.handle = self.sim.createDummy(0.02)
            self.sim.setObjectAlias(vp.handle, f"VP_{vp.name}")
            self.sim.setObjectParent(vp.handle, self.viz_container, True)
            self.sim.setObjectColor(vp.handle, 0, self.sim.colorcomponent_ambient_diffuse, [0, 0, 1])  # 蓝色

        # 立即刷新一次位置
        self.update_all_dummies_pose()

    def update_all_dummies_pose(self):
        """
        核心能力：根据 Receiver 当前的世界姿态，更新所有 Dummy 的位置。
        当转台转动时，必须调用此函数，Dummy 才会跟着转。
        """
        # 1. 获取 Receiver 当前世界矩阵
        mat_list = self.sim.getObjectMatrix(self.rc.handles.receiver, self.rc.handles.world)
        T_world_obj = self._sim_matrix_to_numpy(mat_list)

        # 2. 遍历更新
        for vp in self.viewpoints:
            if vp.handle != -1:
                # T_world_vp = T_world_obj * T_obj_vp
                T_final = T_world_obj @ vp.local_matrix

                # 提取位置和四元数设置给 Dummy
                pos = T_final[:3, 3].tolist()
                quat = self._matrix_to_quat(T_final[:3, :3])

                self.sim.setObjectPose(vp.handle, self.rc.handles.world, pos + list(quat))

    def get_vp_world_pose(self, index: int) -> tuple[Any, tuple[
                                                              float | Any, float | Any, float | Any, float | Any] | Any] | \
                                               tuple[None, None]:
        """获取指定视点当前的实时世界坐标 (Pos, Quat)"""
        if 0 <= index < len(self.viewpoints):
            vp = self.viewpoints[index]
            # 重新计算以保证最新
            mat_list = self.sim.getObjectMatrix(self.rc.handles.receiver, self.rc.handles.world)
            T_world_obj = self._sim_matrix_to_numpy(mat_list)
            T_final = T_world_obj @ vp.local_matrix

            pos = T_final[:3, 3].tolist()
            quat_np = self._matrix_to_quat(T_final[:3, :3])

            if isinstance(quat_np, np.ndarray):
                quat = quat_np.tolist()
            else:
                quat = quat_np

            return pos, quat
        return None, None

    def clear_visuals(self):
        if self.viz_container != -1:
            self.sim.removeObject(self.viz_container)
            self.viz_container = -1

    # --- 轨迹相关 ---
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

    # --- 数学工具 ---
    def _sim_matrix_to_numpy(self, m_list):
        m = np.array(m_list).reshape(3, 4)
        return np.vstack([m, [0, 0, 0, 1]])

    @staticmethod
    def _calc_rotation_matrix_from_dir(direction):
        z = direction / np.linalg.norm(direction)
        up = np.array([0, 0, 1]) if abs(z[2]) < 0.99 else np.array([0, 1, 0])
        x = np.cross(up, z);
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        return np.column_stack((x, y, z))

    @staticmethod
    def _matrix_to_quat(R):
        # 简化的矩阵转四元数 (需确保正确实现，此处略写为占位，建议使用 scipy.spatial.transform 或完整实现)
        # 为了完整性，这里提供一个简单的实现
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) & (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif (R[1, 1] > R[2, 2]):
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S;
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return (qx, qy, qz, qw)

    def clear_trail(self):
        """清除场景中的红色轨迹线"""
        if self.trail_handle != -1:
            try:
                self.sim.removeDrawingObject(self.trail_handle)
                print("[Path] Trail cleared.")
            except Exception as e:
                print(f"[Path] Warning: Failed to clear trail. {e}")
            finally:
                # 无论成功失败，重置句柄，防止二次删除报错
                self.trail_handle = -1

    def clear_all(self):
        """一键清除所有视觉元素（轨迹 + Dummy）"""
        self.clear_trail()
        self.clear_visuals()
