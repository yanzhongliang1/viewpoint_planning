# scan_qt/robot/robot_comm.py
"""
CoppeliaSim ZMQ Remote API 通信层
提供连接管理、对象句柄获取、坐标系变换等基础功能
"""

import numpy as np
import math
from typing import Optional, Dict, Tuple, List
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class RobotCommError(RuntimeError):
    """机器人通信异常"""
    pass


def get_T_W() -> np.ndarray:
    """世界坐标系（单位阵）"""
    return np.eye(4, dtype=float)


class RobotComm:
    """
    封装 CoppeliaSim ZMQ Remote API 通信
    - 连接管理
    - 对象句柄获取
    - 坐标系变换矩阵获取
    - Dummy 对象管理
    """

    # 坐标系标识
    FRAME_WORLD = "W"
    FRAME_TURNTABLE = "J"
    FRAME_OBJECT = "O"
    FRAME_SCANNER = "S"
    FRAME_BASE = "B"

    def __init__(self, host: str = "localhost", port: int = 23000):
        """
        初始化通信对象

        Args:
            host: CoppeliaSim 主机地址
            port: ZMQ Remote API 端口（默认 23000）
        """
        self.host = host
        self.port = port
        self.client: Optional[RemoteAPIClient] = None
        self.sim = None

        # 对象句柄缓存
        self.handles: Dict[str, int] = {}

        # 关节名称
        self.ur5_joint_names = [f"joint{i}" for i in range(1, 7)]
        self.turtle_joint_name = "turtle_joint"

        # Dummy 对象管理
        self.dummy_handles: List[int] = []
        self.trajectory_line_handle: Optional[int] = None

    # ==================== 连接管理 ====================

    def connect(self) -> None:
        """建立与 CoppeliaSim 的连接"""
        try:
            self.client = RemoteAPIClient(self.host, self.port)
            self.sim = self.client.require('sim')

            # 测试连接
            _ = self.sim.getSimulationState()

            # 初始化句柄
            self._init_handles()

            print(f"[RobotComm] 已连接到 CoppeliaSim ({self.host}:{self.port})")

        except Exception as e:
            raise RobotCommError(f"连接失败: {e}")

    def disconnect(self) -> None:
        """断开连接"""
        if self.client is not None:
            try:
                # 清理 Dummy 对象
                self.clear_all_dummies()
                self.clear_trajectory_line()
            except Exception as e:
                print(f"[RobotComm] 清理对象时异常: {e}")

            self.client = None
            self.sim = None
            self.handles.clear()
            print("[RobotComm] 已断开连接")

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.client is not None and self.sim is not None

    def _ensure_connected(self) -> None:
        """确保已连接，否则抛出异常"""
        if not self.is_connected():
            raise RobotCommError("未连接到 CoppeliaSim")

    # ==================== 句柄管理 ====================

    def _init_handles(self) -> None:
        """初始化所有对象句柄"""
        self._ensure_connected()

        # UR5 关节
        for name in self.ur5_joint_names:
            self.handles[name] = self.sim.getObject(f"/{name}")

        # 转台关节
        self.handles[self.turtle_joint_name] = self.sim.getObject(f"/{self.turtle_joint_name}")

        # 关键对象
        self.handles["base"] = self.sim.getObject("/base")
        self.handles["receiver"] = self.sim.getObject("/receiver")
        self.handles["scan"] = self.sim.getObject("/scan")
        self.handles["tip"] = self.sim.getObject("/scan/tip")
        self.handles["target"] = self.sim.getObject("/target")

        print(f"[RobotComm] 已初始化 {len(self.handles)} 个对象句柄")

    def get_handle(self, name: str) -> int:
        """
        获取对象句柄

        Args:
            name: 对象名称（如 "joint1", "base", "scan" 等）

        Returns:
            对象句柄
        """
        if name not in self.handles:
            raise RobotCommError(f"未找到对象: {name}")
        return self.handles[name]

    # ==================== 关节控制 ====================

    def get_joint_position(self, joint_name: str) -> float:
        """
        获取关节角度（弧度）
        Args:
            joint_name: 关节名称
        Returns:
            关节角度（rad）
        """
        self._ensure_connected()
        handle = self.get_handle(joint_name)
        return self.sim.getJointPosition(handle)

    def set_joint_position(self, joint_name: str, position: float) -> None:
        """
        设置关节目标位置（弧度）
        Args:
            joint_name: 关节名称
            position: 目标角度（rad）
        """
        self._ensure_connected()
        handle = self.get_handle(joint_name)
        self.sim.setJointTargetPosition(handle, position)

    def get_all_joint_positions(self) -> Tuple[List[float], float]:
        """
        获取所有关节角度
        Returns:
            (UR5 六轴角度列表, 转台角度)
        """
        ur5_positions = [self.get_joint_position(name) for name in self.ur5_joint_names]
        turtle_position = self.get_joint_position(self.turtle_joint_name)
        return ur5_positions, turtle_position

    def set_all_joint_positions(self, ur5_positions: List[float], turtle_position: float) -> None:
        """
        设置所有关节目标位置

        Args:
            ur5_positions: UR5 六轴目标角度
            turtle_position: 转台目标角度
        """
        for name, pos in zip(self.ur5_joint_names, ur5_positions):
            self.set_joint_position(name, pos)
        self.set_joint_position(self.turtle_joint_name, turtle_position)

    # ==================== 坐标系变换 ====================

    def get_object_pose(self, obj_name: str, relative_to: int = -1) -> np.ndarray:
        """
        获取对象的 4x4 齐次变换矩阵

        Args:
            obj_name: 对象名称
            relative_to: 参考坐标系句柄（-1 表示世界系）

        Returns:
            4x4 齐次变换矩阵
        """
        self._ensure_connected()
        handle = self.get_handle(obj_name)

        # 获取位置和姿态
        position = self.sim.getObjectPosition(handle, relative_to)
        orientation = self.sim.getObjectQuaternion(handle, relative_to)

        # 构造变换矩阵
        T = np.eye(4, dtype=float)
        T[:3, :3] = self._quaternion_to_matrix(orientation)
        T[:3, 3] = position

        return T

    def get_T_WB(self) -> np.ndarray:
        """基座坐标系"""
        return self.get_object_pose("base")

    def get_T_WJ(self) -> np.ndarray:
        """转台坐标系"""
        return self.get_object_pose(self.turtle_joint_name)

    def get_T_WO(self) -> np.ndarray:
        """工件坐标系"""
        return self.get_object_pose("receiver")

    def get_T_WS(self) -> np.ndarray:
        """扫描仪坐标系"""
        return self.get_object_pose("scan")

    def _get_T_by_frame_name(self, frame: str) -> np.ndarray:
        """根据坐标系名称获取变换矩阵"""
        frame = frame.upper()
        if frame == self.FRAME_WORLD:
            return get_T_W()
        elif frame == self.FRAME_BASE:
            return self.get_T_WB()
        elif frame == self.FRAME_TURNTABLE:
            return self.get_T_WJ()
        elif frame == self.FRAME_OBJECT:
            return self.get_T_WO()
        elif frame == self.FRAME_SCANNER:
            return self.get_T_WS()
        else:
            raise RobotCommError(f"未知坐标系: {frame}")

    def transform_point(self, point: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
        """
        坐标系间点变换

        Args:
            point: 3D 点坐标
            from_frame: 源坐标系
            to_frame: 目标坐标系

        Returns:
            变换后的点坐标
        """
        p = np.asarray(point, dtype=float).reshape(3)

        T_W_from = self._get_T_by_frame_name(from_frame)
        T_W_to = self._get_T_by_frame_name(to_frame)

        T_to_from = np.linalg.inv(T_W_to) @ T_W_from

        p_h = np.ones(4, dtype=float)
        p_h[:3] = p
        q_h = T_to_from @ p_h

        return q_h[:3]

    def transform_direction(self, direction: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
        """
        坐标系间方向变换（仅旋转）
        Args:
            direction: 方向向量
            from_frame: 源坐标系
            to_frame: 目标坐标系
        Returns:
            变换后的方向向量
        """
        v = np.asarray(direction, dtype=float).reshape(3)

        T_W_from = self._get_T_by_frame_name(from_frame)
        T_W_to = self._get_T_by_frame_name(to_frame)

        R_W_from = T_W_from[:3, :3]
        R_W_to = T_W_to[:3, :3]

        v_W = R_W_from @ v
        v_to = R_W_to.T @ v_W

        return v_to

    # ==================== Dummy 对象管理 ====================

    def create_dummy(self, position: np.ndarray, name: str = "viewpoint") -> int:
        """
        创建 Dummy 对象
        Args:
            position: 位置 [x, y, z]
            name: 对象名称
        Returns:
            Dummy 句柄
        """
        self._ensure_connected()

        # 创建 Dummy
        dummy_handle = self.sim.createDummy(0.02)  # 2cm 大小
        self.sim.setObjectAlias(dummy_handle, name)

        # 设置位置
        self.sim.setObjectPosition(dummy_handle, position, -1)

        # 设置颜色（绿色）
        self.sim.setShapeColor(dummy_handle, None, 0, [0.0, 1.0, 0.0])

        self.dummy_handles.append(dummy_handle)

        return dummy_handle

    def create_dummies_from_viewpoints(self, viewpoints: List[Tuple[str, np.ndarray, np.ndarray]]) -> List[int]:
        """
        从视点列表批量创建 Dummy
        Args:
            viewpoints: [(name, position, direction), ...]
        Returns:
            Dummy 句柄列表
        """
        handles = []
        for name, position, _ in viewpoints:
            handle = self.create_dummy(position, name)
            handles.append(handle)

        print(f"[RobotComm] 已创建 {len(handles)} 个视点 Dummy")
        return handles

    def clear_all_dummies(self) -> None:
        """清除所有创建的 Dummy"""
        self._ensure_connected()

        for handle in self.dummy_handles:
            try:
                self.sim.removeObject(handle)
            except Exception as e:
                print(f"[RobotComm] 删除 Dummy 失败: {e}")

        self.dummy_handles.clear()
        print("[RobotComm] 已清除所有 Dummy")

    # ==================== 轨迹可视化 ====================

    def create_trajectory_line(self, points: List[np.ndarray], color: List[float] = None) -> int:
        """
        创建轨迹线
        Args:
            points: 轨迹点列表
            color: RGB 颜色 [r, g, b]
        Returns:
            轨迹线句柄
        """
        self._ensure_connected()

        if color is None:
            color = [0.0, 1.0, 0.0]  # 默认绿色

        # 清除旧轨迹
        self.clear_trajectory_line()

        # 创建线条对象
        line_data = []
        for i in range(len(points) - 1):
            line_data.extend(points[i])
            line_data.extend(points[i + 1])

        self.trajectory_line_handle = self.sim.addDrawingObject(
            self.sim.drawing_lines,  # 线条类型
            2.0,  # 线宽
            0.0,  # 生命周期（0 表示永久）
            -1,  # 父对象
            99999,  # 最大点数
            color + [1.0] * 3  # 颜色 + alpha
        )

        # 添加线段
        for i in range(0, len(line_data), 6):
            self.sim.addDrawingObjectItem(
                self.trajectory_line_handle,
                line_data[i:i + 6]
            )

        return self.trajectory_line_handle

    def clear_trajectory_line(self) -> None:
        """清除轨迹线"""
        if self.trajectory_line_handle is not None:
            try:
                self.sim.removeDrawingObject(self.trajectory_line_handle)
            except Exception:
                pass
            self.trajectory_line_handle = None

    # ==================== 工具函数 ====================

    @staticmethod
    def _quaternion_to_matrix(q: List[float]) -> np.ndarray:
        """
        四元数转旋转矩阵
        Args:
            q: 四元数 [x, y, z, w]
        Returns:
            3x3 旋转矩阵
        """
        x, y, z, w = q

        R = np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x ** 2 + y ** 2)]
        ], dtype=float)

        return R

    @staticmethod
    def matrix_to_pos_euler(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        4x4 矩阵分解为位置和欧拉角
        Args:
            T: 4x4 齐次变换矩阵
        Returns:
            (位置, 欧拉角 XYZ)
        """
        R = T[:3, :3]
        t = T[:3, 3]

        # 提取欧拉角（XYZ 顺序）
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

        if sy > 1e-6:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0.0

        euler = np.array([x, y, z], dtype=float)

        return t, euler