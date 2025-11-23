# scan_qt/robot/robot_model.py
"""
机器人参数模型
定义关节限位、速度/加速度约束、Home 位姿等
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class JointLimit:
    """关节限位"""
    min_deg: float
    max_deg: float

    @property
    def min_rad(self) -> float:
        return math.radians(self.min_deg)

    @property
    def max_rad(self) -> float:
        return math.radians(self.max_deg)


@dataclass
class RobotModel:
    """
    UR5 + 转台机器人模型
    """
    # 关节名称
    ur5_joint_names: List[str] = field(default_factory=lambda: [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ])
    turtle_joint_name: str = "turtle_joint"

    # 关节限位（角度制）
    joint_limits_deg: Dict[str, JointLimit] = field(default_factory=lambda: {
        "joint1": JointLimit(-360.0, 360.0),
        "joint2": JointLimit(-360.0, 360.0),
        "joint3": JointLimit(-360.0, 360.0),
        "joint4": JointLimit(-360.0, 360.0),
        "joint5": JointLimit(-360.0, 360.0),
        "joint6": JointLimit(-360.0, 360.0),
        "turtle_joint": JointLimit(-180.0, 180.0),
    })

    # 运动学约束（弧度制）
    max_vel_ur5_rad: List[float] = field(default_factory=lambda: [
                                                                     math.radians(120.0)
                                                                 ] * 6)

    max_acc_ur5_rad: List[float] = field(default_factory=lambda: [
                                                                     math.radians(40.0)
                                                                 ] * 6)

    max_vel_turtle_rad: float = math.radians(60.0)
    max_acc_turtle_rad: float = math.radians(30.0)

    # Home 位姿（弧度制）
    home_ur5_rad: List[float] = field(default_factory=lambda: [0.0] * 6)
    home_turtle_rad: float = 0.0

    # ==================== 工具方法 ====================

    def clamp_joint_rad(self, name: str, value_rad: float) -> float:
        """限制关节角度在范围内"""
        if name not in self.joint_limits_deg:
            return value_rad

        lim = self.joint_limits_deg[name]
        return max(lim.min_rad, min(lim.max_rad, value_rad))

    def clamp_ur5_config_rad(self, q_rad: List[float]) -> List[float]:
        """限制 UR5 配置"""
        return [
            self.clamp_joint_rad(name, v)
            for name, v in zip(self.ur5_joint_names, q_rad)
        ]

    def normalize_angle(self, angle_rad: float) -> float:
        """归一化角度到 [-π, π]"""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def deg_to_rad(deg: float) -> float:
        return math.radians(deg)

    @staticmethod
    def rad_to_deg(rad: float) -> float:
        return math.degrees(rad)
