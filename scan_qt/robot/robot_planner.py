# scan_qt/robot/robot_planner.py
import time
import math
from typing import List, Tuple

import numpy as np

from scan_qt.coppeliasim import sim
from scan_qt.robot.robot_model import RobotModel
from scan_qt.robot.robot_comm import RobotComm, RobotCommError


class MoveToConfig1D:
    """
    1 维关节 q0 -> q1 的时间轨迹
    使用加速度受限的梯形 / 三角速度曲线
    maxVel, maxAccel, maxJerk: 标称上限（jerk 这里未显式使用）
    """

    def __init__(self, q0, q1, maxVel, maxAccel, maxJerk, dt=0.005):
        self.q0 = float(q0)
        self.q1 = float(q1)
        self.maxVel = abs(maxVel)
        self.maxAccel = abs(maxAccel)
        self.maxJerk = abs(maxJerk)
        self.dt = dt

        self.direction = 1.0 if self.q1 >= self.q0 else -1.0
        self.D = abs(self.q1 - self.q0)  # 总位移

        if self.D < 1e-8:
            # 基本不动
            self.t_acc = 0.0
            self.s_acc = 0.0
            self.t_flat = 0.0
            self.s_flat = 0.0
            self.t_total = 0.0
            self.time = 0.0
            return

        # 加速到最大速度所需时间和位移
        self.t_acc = self.maxVel / self.maxAccel
        self.s_acc = 0.5 * self.maxAccel * self.t_acc ** 2

        # 判断是三角速度还是梯形速度
        if 2.0 * self.s_acc >= self.D:
            # 三角速度：到不了最大速度
            self.t_acc = math.sqrt(self.D / self.maxAccel)
            self.s_acc = 0.5 * self.maxAccel * self.t_acc ** 2
            self.t_flat = 0.0
            self.s_flat = 0.0
        else:
            # 梯形速度
            self.s_flat = self.D - 2.0 * self.s_acc
            self.t_flat = self.s_flat / self.maxVel

        self.t_total = 2.0 * self.t_acc + self.t_flat
        self.time = 0.0

    def step(self):
        """
        返回下一时刻的关节角，不再有值时返回 None
        """
        if self.time > self.t_total:
            return None

        t = self.time

        # 加速段
        if t < self.t_acc:
            s = 0.5 * self.maxAccel * t * t

        # 匀速段
        elif t < self.t_acc + self.t_flat:
            s = self.s_acc + self.maxVel * (t - self.t_acc)

        # 减速段
        else:
            td = t - self.t_acc - self.t_flat
            s = self.s_acc + self.s_flat + (self.maxVel * td - 0.5 * self.maxAccel * td * td)

        q = self.q0 + self.direction * s
        self.time += self.dt
        return q


class RobotPlanner:
    """
    基于 RobotComm + RobotModel 的七轴规划与执行：
    - 读取当前关节
    - 关节空间目标配置规划并下发（七轴联合）
    - home 动作
    - 简单 yaw 分配示例（总 yaw = joint1 + turtle）
    """

    def __init__(self, model: RobotModel, comm: RobotComm, dt: float = 0.005):
        self.model = model
        self.comm = comm
        self.dt = dt

        if not self.comm.is_connected():
            raise RobotCommError("RobotPlanner 初始化时 RobotComm 未连接")

        # 简单缓存当前关节，方便某些规划策略
        self.q_ur5, self.q_turtle = self.read_current_config()

    # ------------- 关节状态读取 -------------

    def read_current_config(self) -> Tuple[List[float], float]:
        """
        读取当前 UR5 六轴 + 转台关节角度（rad）。
        """
        if not self.comm.is_connected():
            raise RobotCommError("未连接 CoppeliaSim，无法读取关节状态")

        q_ur5 = []
        for jh in self.comm.ur5_joints:
            err, pos = sim.simxGetJointPosition(
                self.comm.client_id, jh, sim.simx_opmode_blocking
            )
            if err != sim.simx_return_ok:
                raise RobotCommError(f"simxGetJointPosition(UR5) 失败, err={err}")
            q_ur5.append(pos)

        err, pos_turtle = sim.simxGetJointPosition(
            self.comm.client_id, self.comm.handle_turtle, sim.simx_opmode_blocking
        )
        if err != sim.simx_return_ok:
            raise RobotCommError(f"simxGetJointPosition(turtle_joint) 失败, err={err}")

        self.q_ur5 = q_ur5[:]
        self.q_turtle = pos_turtle
        return q_ur5, pos_turtle

    # ------------- 七轴关节空间规划 -------------

    def move_to_config(self,
                       q_target_ur5: List[float],
                       turtle_target: float,
                       sync: bool = True):
        """
        七轴关节空间运动到目标配置：
        - 输入为弧度制
        - 自动进行关节限幅
        - 简单使用 MoveToConfig1D 做梯形/三角速度轨迹
        - 当前版本在主线程中执行，会阻塞 UI（后续可升级为 QThread）
        """
        if not self.comm.is_connected():
            raise RobotCommError("未连接 CoppeliaSim，无法执行关节运动")

        # 当前状态
        q0_ur5, q0_turtle = self.read_current_config()

        # 限幅
        q_target_ur5 = self.model.clamp_ur5_config_rad(q_target_ur5)
        turtle_target = self.model.clamp_turtle_rad(turtle_target)

        # 构造规划器
        planners = []
        for q0, q1, v, a, j in zip(
            q0_ur5,
            q_target_ur5,
            self.model.max_vel_ur5_rad,
            self.model.max_acc_ur5_rad,
            self.model.max_jerk_ur5_rad
        ):
            planners.append(MoveToConfig1D(q0, q1, v, a, j, self.dt))

        turtle_planner = MoveToConfig1D(
            q0_turtle,
            turtle_target,
            self.model.max_vel_turtle_rad,
            self.model.max_acc_turtle_rad,
            self.model.max_jerk_turtle_rad,
            self.dt,
        )

        active = True
        while active:
            active = False
            q_cmd_ur5 = []

            # 六轴 UR5
            for p in planners:
                v = p.step()
                q_cmd_ur5.append(v)
                if v is not None:
                    active = True

            # 转台
            q_cmd_turtle = turtle_planner.step()
            if q_cmd_turtle is not None:
                active = True

            # 发送到 UR5 关节
            for val, jh in zip(q_cmd_ur5, self.comm.ur5_joints):
                if val is not None:
                    sim.simxSetJointTargetPosition(
                        self.comm.client_id, jh, val, sim.simx_opmode_oneshot
                    )

            # 发送到转台关节
            if q_cmd_turtle is not None:
                sim.simxSetJointTargetPosition(
                    self.comm.client_id, self.comm.handle_turtle, q_cmd_turtle, sim.simx_opmode_oneshot
                )

            time.sleep(self.dt)

        # 更新缓存
        self.q_ur5 = q_target_ur5[:]
        self.q_turtle = turtle_target

        if sync:
            print("[RobotPlanner] 已到达目标配置")

    # ------------- Home 位姿 -------------

    def go_home(self):
        """
        回到 RobotModel 定义的 home 位姿。
        """
        self.move_to_config(self.model.home_ur5_rad, self.model.home_turtle_rad)

    # ------------- 简单 yaw 分配示例 -------------

    def plan_yaw_split(self, desired_yaw_rad: float) -> Tuple[List[float], float]:
        """
        一个简单的“总 yaw = joint1 + turtle_joint”分配策略：
        - 希望 joint1 尽量靠近 0，更多 yaw 分配给转台
        - 将转台角限制在 [-90°, 90°] 范围内
        仅为示例，不考虑奇异/碰撞等。
        """
        # 当前其它关节保持不变
        q_now, turtle_now = self.read_current_config()

        psi = desired_yaw_rad  # 总 yaw

        limit = math.radians(90.0)
        turtle_target = max(-limit, min(limit, psi))
        joint1_target = psi - turtle_target

        ur5_target = q_now[:]
        ur5_target[0] = joint1_target  # 只调第一关节

        # 限幅
        ur5_target = self.model.clamp_ur5_config_rad(ur5_target)
        turtle_target = self.model.clamp_turtle_rad(turtle_target)

        return ur5_target, turtle_target

    def go_to_yaw(self, desired_yaw_rad: float):
        """
        使用 plan_yaw_split 规划 yaw，然后执行七轴运动。
        """
        ur5_target, turtle_target = self.plan_yaw_split(desired_yaw_rad)
        print(
            f"[RobotPlanner] yaw 分配: total={math.degrees(desired_yaw_rad):.1f}deg, "
            f"joint1={math.degrees(ur5_target[0]):.1f}deg, "
            f"turtle={math.degrees(turtle_target):.1f}deg"
        )
        self.move_to_config(ur5_target, turtle_target)
