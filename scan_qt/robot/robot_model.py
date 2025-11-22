# scan_qt/robot/robot_model.py
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass
class JointLimit:
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
    UR5 + 转台（turtle_joint）的静态参数：
    - 关节命名
    - 关节限位
    - 默认最大速度/加速度/jerk
    - home 位姿
    """

    # UR5 六轴关节名称（必须与 CoppeliaSim 中一致）
    ur5_joint_names: List[str] = field(default_factory=lambda: [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ])
    # 转台关节名称
    turtle_joint_name: str = "turtle_joint"

    # 关节限位（角度制）
    joint_limits_deg: Dict[str, JointLimit] = field(default_factory=lambda: {
        # 这里使用比较宽松的 UR5 范围（仅示意，可根据实际调整）
        "joint1": JointLimit(-360.0, 720.0),
        "joint2": JointLimit(-360.0, 720.0),
        "joint3": JointLimit(-360.0, 720.0),
        "joint4": JointLimit(-360.0, 720.0),
        "joint5": JointLimit(-360.0, 720.0),
        "joint6": JointLimit(-360.0, 720.0),
        # 转台：默认 [-180, 180]，可根据实际机械结构调整
        "turtle_joint": JointLimit(-180.0, 180.0),
    })

    # 默认最大速度/加速度/jerk（弧度制）
    # 这里用和你之前测试脚本类似的值
    max_vel_ur5_rad: List[float] = field(default_factory=lambda: [
        math.radians(180.0)
    ] * 6)
    max_acc_ur5_rad: List[float] = field(default_factory=lambda: [
        math.radians(40.0)
    ] * 6)
    max_jerk_ur5_rad: List[float] = field(default_factory=lambda: [
        math.radians(80.0)
    ] * 6)

    max_vel_turtle_rad: float = math.radians(90.0)
    max_acc_turtle_rad: float = math.radians(30.0)
    max_jerk_turtle_rad: float = math.radians(60.0)

    # home 位姿（弧度制）
    home_ur5_rad: List[float] = field(default_factory=lambda: [0.0] * 6)
    home_turtle_rad: float = 0.0

    def clamp_joint_rad(self, name: str, value_rad: float) -> float:
        """
        将某关节角度（rad）限制在 joint_limits 内。
        """
        if name not in self.joint_limits_deg:
            return value_rad
        lim = self.joint_limits_deg[name]
        return max(lim.min_rad, min(lim.max_rad, value_rad))

    def clamp_ur5_config_rad(self, q_rad: List[float]) -> List[float]:
        """
        对 UR5 六轴配置进行限幅。
        """
        res = []
        for name, v in zip(self.ur5_joint_names, q_rad):
            res.append(self.clamp_joint_rad(name, v))
        return res

    def clamp_turtle_rad(self, v_rad: float) -> float:
        return self.clamp_joint_rad(self.turtle_joint_name, v_rad)

    def deg_to_rad_ur5(self, q_deg: List[float]) -> List[float]:
        return [math.radians(v) for v in q_deg]

    def rad_to_deg_ur5(self, q_rad: List[float]) -> List[float]:
        return [math.degrees(v) for v in q_rad]

    def deg_to_rad_turtle(self, v_deg: float) -> float:
        return math.radians(v_deg)

    def rad_to_deg_turtle(self, v_rad: float) -> float:
        return math.degrees(v_rad)
