import scan_qt.coppeliasim.simConst as simConst
import scan_qt.coppeliasim.sim as sim

import time
import math

def rad(deg):
    """角度制 -> 弧度制"""
    return deg * math.pi / 180.0


# =====================================================
# 单关节梯形 / 三角速度轨迹规划器
# =====================================================
class MoveToConfig1D:
    """
    1 维关节 q0 -> q1 的时间轨迹
    使用加速度受限的梯形 / 三角速度曲线
    maxVel, maxAccel, maxJerk: 标称上限（jerk 这里未显式使用）
    """

    def __init__(self, q0, q1, maxVel, maxAccel, maxJerk, dt=0.005):
        self.q0 = q0
        self.q1 = q1
        self.maxVel = abs(maxVel)
        self.maxAccel = abs(maxAccel)
        self.maxJerk = abs(maxJerk)
        self.dt = dt

        self.direction = 1.0 if q1 >= q0 else -1.0
        self.D = abs(q1 - q0)  # 总位移

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


# =====================================================
# 七轴控制器：UR5 六轴 + 转台 turtle_joint
# =====================================================
class SevenAxisController:
    def __init__(self, client_id):
        self.client_id = client_id

        self.ur5_joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        self.ur5_joints = []
        for name in self.ur5_joint_names:
            err, h = sim.simxGetObjectHandle(client_id, name, sim.simx_opmode_blocking)
            if err != sim.simx_return_ok:
                raise RuntimeError(f"无法获取关节: {name}")
            self.ur5_joints.append(h)

        # 转台关节
        err, self.turtle_joint = sim.simxGetObjectHandle(
            client_id, "turtle_joint", sim.simx_opmode_blocking
        )
        if err != sim.simx_return_ok:
            raise RuntimeError("无法获取转台关节 turtle_joint")

        # 当前关节状态
        self.q_ur5 = self.get_ur5_joint_positions()
        self.q_turtle = self.get_turtle_position()

    # ---------- 状态读取 ----------

    def get_ur5_joint_positions(self):
        q = []
        for jh in self.ur5_joints:
            err, pos = sim.simxGetJointPosition(
                self.client_id, jh, sim.simx_opmode_blocking
            )
            if err != sim.simx_return_ok:
                raise RuntimeError("simxGetJointPosition(UR5) 失败")
            q.append(pos)
        return q

    def get_turtle_position(self):
        err, pos = sim.simxGetJointPosition(
            self.client_id, self.turtle_joint, sim.simx_opmode_blocking
        )
        if err != sim.simx_return_ok:
            raise RuntimeError("simxGetJointPosition(turtle_joint) 失败")
        return pos

    # ---------- 七轴联合规划 & 运动 ----------

    def move_7axis(
        self,
        ur5_target,
        turtle_target,
        maxVel_ur5,
        maxAccel_ur5,
        maxJerk_ur5,
        maxVel_turtle,
        maxAccel_turtle,
        maxJerk_turtle,
        dt=0.005,
    ):
        """
        7 轴联合运动：
        - UR5 六轴目标: ur5_target (list[6])
        - 转台目标: turtle_target (float)
        - 各自的 maxVel/maxAccel/maxJerk
        - 通过 MoveToConfig1D 给每个关节生成时间轨迹
        - 每个 dt 调用 simxSetJointTargetPosition 对 dynamic + position control 关节下发目标
        """

        # 当前状态
        q0_ur5 = self.q_ur5[:]
        q0_turtle = self.q_turtle

        # 为 6 个 UR5 关节创建规划器
        planners = []
        for q0, q1, v, a, j in zip(
            q0_ur5, ur5_target, maxVel_ur5, maxAccel_ur5, maxJerk_ur5
        ):
            planners.append(MoveToConfig1D(q0, q1, v, a, j, dt))

        # 为转台创建规划器（第 7 轴）
        turtle_planner = MoveToConfig1D(
            q0_turtle, turtle_target, maxVel_turtle, maxAccel_turtle, maxJerk_turtle, dt
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

            # 发送到 UR5 关节：target position（dynamic + position control）
            for val, jh in zip(q_cmd_ur5, self.ur5_joints):
                if val is not None:
                    sim.simxSetJointTargetPosition(
                        self.client_id, jh, val, sim.simx_opmode_oneshot
                    )

            # 发送到转台关节
            if q_cmd_turtle is not None:
                sim.simxSetJointTargetPosition(
                    self.client_id, self.turtle_joint, q_cmd_turtle, sim.simx_opmode_oneshot
                )

            time.sleep(dt)

        # 更新内部状态
        self.q_ur5 = ur5_target[:]
        self.q_turtle = turtle_target

    # ---------- 一个简单的“转台 + 六轴联合规划策略”示例 ----------

    def plan_yaw_split(self, desired_yaw):
        """
        非严格 IK 的简单示例：
        假设 "系统总偏航角" ≈ UR5_joint1 + turtle_joint
        给定一个期望的总 yaw（绕 Z 轴），
        采用一个简单的分配策略：
            - 尽量让 UR5_joint1 接近 0（避免大幅度旋转）
            - 把大部分 yaw 分配给转台 turtle_joint

        这里给出一个非常简单的策略：
            turtle = clamp(desired_yaw, [-90°, 90°])
            joint1 = desired_yaw - turtle
        其他关节不动，仅做演示用。
        """
        # 当前其它关节先保持不变
        q_now = self.q_ur5[:]
        turtle_now = self.q_turtle

        # 期望 yaw（弧度）
        psi = desired_yaw

        # 把转台角限制在 [-90°, 90°]
        limit = rad(90)
        turtle_target = max(-limit, min(limit, psi))

        # 将剩余的 yaw 分配给 UR5_joint1
        joint1_target = psi - turtle_target

        ur5_target = q_now[:]
        ur5_target[0] = joint1_target  # 只改第一关节，演示用

        return ur5_target, turtle_target


# =====================================================
#            测试用例：转台 + 六轴联合运动
# =====================================================
def main():
    # 断开旧连接
    sim.simxFinish(-1)

    # 连接 CoppeliaSim
    client_id = sim.simxStart("127.0.0.1", 19997, True, True, 5000, 5)
    if client_id == -1:
        print("无法连接到 CoppeliaSim remote API")
        return
    print("成功连接 CoppeliaSim")

    # 启动仿真
    sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)

    ctrl = SevenAxisController(client_id)

    # ---------------------------
    # 测试用例 1：只做六轴运动（转台不动）
    # ---------------------------
    vel = rad(180)
    accel = rad(40)
    jerk = rad(80)

    maxVel_ur5 = [vel] * 6
    maxAccel_ur5 = [accel] * 6
    maxJerk_ur5 = [jerk] * 6

    maxVel_turtle = rad(90)
    maxAccel_turtle = rad(30)
    maxJerk_turtle = rad(60)

    print("Test 1: 只动六轴，转台保持不动")

    # 目标姿态示例：相当于你之前 Lua 脚本里的第一个位置
    ur5_target1 = [rad(90), rad(90), rad(-90), rad(90), rad(90), rad(90)]
    turtle_target1 = ctrl.q_turtle  # 不动

    ctrl.move_7axis(
        ur5_target1,
        turtle_target1,
        maxVel_ur5,
        maxAccel_ur5,
        maxJerk_ur5,
        maxVel_turtle,
        maxAccel_turtle,
        maxJerk_turtle,
    )

    time.sleep(1.0)

    # ---------------------------
    # 测试用例 2：转台 + 六轴联合规划（分配 yaw）
    # ---------------------------
    print("Test 2: 转台 + 六轴联合规划 (Yaw 拆分)")

    # 假设我们希望总偏航角为 +150 度
    desired_yaw = rad(150)

    ur5_target2, turtle_target2 = ctrl.plan_yaw_split(desired_yaw)
    print(
        f"期望总 yaw = 150deg, 规划结果: "
        f"UR5_joint1={ur5_target2[0]:.3f} rad, turtle_joint={turtle_target2:.3f} rad"
    )

    ctrl.move_7axis(
        ur5_target2,
        turtle_target2,
        maxVel_ur5,
        maxAccel_ur5,
        maxJerk_ur5,
        maxVel_turtle,
        maxAccel_turtle,
        maxJerk_turtle,
    )

    time.sleep(1.0)

    # ---------------------------
    # 测试用例 3：返回初始姿态 + 转台回零
    # ---------------------------
    print("Test 3: 回到初始位姿 + 转台回零")

    ur5_home = [0.0] * 6
    turtle_home = 0.0

    ctrl.move_7axis(
        ur5_home,
        turtle_home,
        maxVel_ur5,
        maxAccel_ur5,
        maxJerk_ur5,
        maxVel_turtle,
        maxAccel_turtle,
        maxJerk_turtle,
    )

    # 仿真结束
    sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
    sim.simxFinish(client_id)
    print("测试结束。")


if __name__ == "__main__":
    main()