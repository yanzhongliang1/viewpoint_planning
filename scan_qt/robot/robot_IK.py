# scan_qt/robot/robot_ik.py
"""
机器人逆运动学求解
基于 CoppeliaSim 的 IK 组和 tip-target 机制
"""
import numpy as np
import math
from typing import Optional, List, Tuple
from scan_qt.robot.robot_comm import RobotComm, RobotCommError
from scan_qt.robot.robot_model import RobotModel


class RobotIKError(Exception):
    """逆运动学求解异常"""
    pass


class RobotIK:
    """
    机器人逆运动学求解器

    使用 CoppeliaSim 的 IK 功能:
    - tip: 扫描仪末端点（/scan/tip）
    - target: 目标位姿对象（/target）
    - IK Group: 需要在场景中配置
    """

    def __init__(self, comm: RobotComm, model: RobotModel):
        """
        初始化 IK 求解器
        Args:
            comm: 通信对象
            model: 机器人模型
        """
        self.comm = comm
        self.model = model

        # IK 相关句柄
        self.tip_handle: Optional[int] = None
        self.target_handle: Optional[int] = None
        self.ik_group_handle: Optional[int] = None

        self._init_ik()

    def _init_ik(self) -> None:
        """初始化 IK 组"""
        if not self.comm.is_connected():
            raise RobotIKError("通信未连接")

        try:
            self.tip_handle = self.comm.get_handle("tip")
            self.target_handle = self.comm.get_handle("target")

            # 获取 IK 组（需要在场景中预先创建）
            # 名称通常为 "UR5_IK" 或类似
            try:
                self.ik_group_handle = self.comm.sim.getIkGroupHandle("UR5_IK")
            except Exception:
                print("[RobotIK] 警告: 未找到 IK 组 'UR5_IK'，将尝试创建")
                self._create_ik_group()

            print("[RobotIK] IK 求解器初始化完成")

        except Exception as e:
            raise RobotIKError(f"IK 初始化失败: {e}")

    def _create_ik_group(self) -> None:
        """创建 IK 组（如果不存在）"""
        try:
            # 创建 IK 组
            self.ik_group_handle = self.comm.sim.createIkGroup()
            self.comm.sim.setObjectAlias(self.ik_group_handle, "UR5_IK")

            # 创建 IK 元素（tip -> target）
            ik_element = self.comm.sim.createIkElement(
                self.ik_group_handle,
                self.tip_handle,
                self.target_handle
            )

            # 设置 IK 参数
            self.comm.sim.setIkElementProperties(
                self.ik_group_handle,
                ik_element,
                constraints=self.comm.sim.ik_x_constraint |
                            self.comm.sim.ik_y_constraint |
                            self.comm.sim.ik_z_constraint |
                            self.comm.sim.ik_alpha_beta_constraint |
                            self.comm.sim.ik_gamma_constraint
            )

            print("[RobotIK] 已创建 IK 组")

        except Exception as e:
            raise RobotIKError(f"创建 IK 组失败: {e}")

    # ==================== 核心求解方法 ====================

    def solve_ik(self, target_pose: np.ndarray,
                 initial_config: Optional[List[float]] = None,
                 max_iterations: int = 100,
                 precision: float = 0.001) -> Tuple[bool, List[float], float]:
        """
        求解逆运动学
        Args:
            target_pose: 目标位姿（4x4 齐次变换矩阵，世界坐标系）
            initial_config: 初始关节配置（可选）
            max_iterations: 最大迭代次数
            precision: 精度阈值（米）
        Returns:
            (成功标志, UR5 关节角度列表, 转台角度)
        """
        if not self.comm.is_connected():
            raise RobotIKError("通信未连接")

        try:
            # 1. 设置初始配置
            if initial_config is not None:
                self.comm.set_all_joint_positions(
                    initial_config[:6],
                    initial_config[6] if len(initial_config) > 6 else 0.0
                )

            # 2. 设置目标位姿
            position = target_pose[:3, 3].tolist()

            # 将旋转矩阵转换为四元数
            quaternion = self._matrix_to_quaternion(target_pose[:3, :3])

            self.comm.sim.setObjectPosition(self.target_handle, position, -1)
            self.comm.sim.setObjectQuaternion(self.target_handle, quaternion, -1)

            # 3. 执行 IK 求解
            result = self.comm.sim.handleIkGroup(self.ik_group_handle)

            # 4. 检查结果
            if result == self.comm.sim.ikresult_success:
                # 读取求解后的关节角度
                ur5_config, turtle_config = self.comm.get_all_joint_positions()

                # 验证精度
                current_pose = self.comm.get_object_pose("tip")
                position_error = np.linalg.norm(
                    current_pose[:3, 3] - target_pose[:3, 3]
                )

                if position_error < precision:
                    return True, ur5_config, turtle_config
                else:
                    print(f"[RobotIK] 位置误差过大: {position_error * 1000:.2f} mm")
                    return False, ur5_config, turtle_config
            else:
                print(f"[RobotIK] IK 求解失败，结果码: {result}")
                return False, [], 0.0

        except Exception as e:
            raise RobotIKError(f"IK 求解异常: {e}")

    def solve_ik_for_viewpoint(self, position: np.ndarray,
                               direction: np.ndarray,
                               approach_distance: float = 0.0) -> Tuple[bool, List[float], float]:
        """
        为视点求解 IK

        Args:
            position: 视点位置（世界坐标系）
            direction: 扫描方向（指向工件）
            approach_distance: 接近距离（米，正值表示远离工件）

        Returns:
            (成功标志, UR5 关节角度, 转台角度)
        """
        # 1. 计算实际目标位置（考虑接近距离）
        direction_norm = direction / np.linalg.norm(direction)
        actual_position = position - direction_norm * approach_distance

        # 2. 构造目标姿态（Z 轴指向工件）
        z_axis = -direction_norm  # 扫描仪 Z 轴指向工件

        # 选择合适的 X 轴（避免奇异）
        if abs(z_axis[2]) < 0.9:
            x_axis = np.cross([0, 0, 1], z_axis)
        else:
            x_axis = np.cross([1, 0, 0], z_axis)

        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        # 3. 构造变换矩阵
        target_pose = np.eye(4, dtype=float)
        target_pose[:3, 0] = x_axis
        target_pose[:3, 1] = y_axis
        target_pose[:3, 2] = z_axis
        target_pose[:3, 3] = actual_position

        # 4. 求解 IK
        return self.solve_ik(target_pose)

    def solve_ik_batch(self, viewpoints: List[Tuple[str, np.ndarray, np.ndarray]],
                       approach_distance: float = 0.0) -> List[Tuple[str, bool, List[float], float]]:
        """
        批量求解视点 IK

        Args:
            viewpoints: [(name, position, direction), ...]
            approach_distance: 接近距离

        Returns:
            [(name, success, ur5_config, turtle_config), ...]
        """
        results = []

        for name, position, direction in viewpoints:
            success, ur5_config, turtle_config = self.solve_ik_for_viewpoint(
                position, direction, approach_distance
            )

            results.append((name, success, ur5_config, turtle_config))

            if success:
                print(f"[RobotIK] {name} 求解成功")
            else:
                print(f"[RobotIK] {name} 求解失败")

        success_count = sum(1 for _, s, _, _ in results if s)
        print(f"[RobotIK] 批量求解完成: {success_count}/{len(viewpoints)} 成功")

        return results

    # ==================== 工具方法 ====================

    @staticmethod
    def _matrix_to_quaternion(R: np.ndarray) -> List[float]:
        """
        旋转矩阵转四元数

        Args:
            R: 3x3 旋转矩阵

        Returns:
            四元数 [x, y, z, w]
        """
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return [x, y, z, w]
