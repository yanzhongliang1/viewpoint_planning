from typing import Optional, Tuple, List
import math

# 引用你的 RobotComm 类和常量
from scan_qt.test.robot_comm import RobotComm, Frames


class RobotIK:
    """
    基于 simIK 的逆运动学解算器。
    专为 UR5 + ZMQ Remote API 设计。
    """

    def __init__(self, rc: RobotComm):
        self.rc = rc
        self.sim = rc.sim
        self.simIK = rc.simIK

        # 初始化 IK 环境
        self.ik_env = self.simIK.createEnvironment()
        self.ik_group = self.simIK.createGroup(self.ik_env)

        # 配置 IK 链
        self._setup_ur5_chain()

        if self.rc.verbose:
            print("[RobotIK] IK Environment & Group initialized.")

    def _setup_ur5_chain(self):
        """内部配置：建立 Base -> Tip 的 IK 约束链"""

        # 设置 IK 模式：同时约束位置(X,Y,Z)和姿态(Alpha,Beta,Gamma)
        # 对应 simIK.constraint_pose (位姿全约束)
        # 注意：ZMQ API 中常量通常在 simIK 命名空间下
        constraints = self.simIK.constraint_position | self.simIK.constraint_orientation

        # 添加从场景获取的元素
        # 逻辑：Tip (Follower) 将尝试重合于 Target (Leader)
        self.ik_element = self.simIK.addElementFromScene(
            self.ik_env,
            self.ik_group,
            self.rc.handles.base,  # 链的基座
            self.rc.handles.tip,  # 链的末端 (Follower)
            self.rc.handles.target,  # 目标 Dummy (Leader)
            constraints
        )

        # 设置解算方法：DLS (Damped Least Squares) 通常比 PseudoInverse 更稳定
        # 参数: env, group, method, damping, maxIterations
        self.simIK.setGroupCalculation(
            self.ik_env,
            self.ik_group,
            self.simIK.method_damped_least_squares,
            0.1,  # 阻尼系数
            100  # 最大迭代次数
        )

    def solve(self,
              pos: Tuple[float, float, float],
              quat: Tuple[float, float, float, float],
              ref_frame: str = Frames.WORLD,
              restore_if_fail: bool = True) -> Optional[Tuple[float, ...]]:

        # 1. 【记录起点】
        start_angles = self.rc.get_ur5_angles()

        try:
            # 2. 设置 Target 位置
            h_ref = self.rc._resolve_frame(ref_frame)
            self.sim.setObjectPose(self.rc.handles.target, h_ref, list(pos) + list(quat))

            # 3. 运行解算 (syncWorlds=True 会导致机器人瞬移到目标姿态)
            result, _, _ = self.simIK.handleGroup(
                self.ik_env,
                self.ik_group,
                {'syncWorlds': True, 'allowError': True}
            )

            # 1 = simIK.result_success
            # 2 = simIK.result_success_with_error (例如使用了阻尼)
            success = (result == 1) or (result == 2)

            if success:
                # 4. 【获取答案】此时机器人处于目标姿态，读取角度
                solved_angles = self.rc.get_ur5_angles()

                # 5. 【关键修正：复位】
                # 无论是否成功，为了让物理引擎能演示"移动过程"，
                # 我们必须把机器人瞬间搬回起点。
                self.rc.set_ur5_angles(start_angles, instant=True)

                return solved_angles
            else:
                if self.rc.verbose:
                    print(f"[RobotIK] Failed. Result code: {result}")
                # 失败了也复位
                if restore_if_fail:
                    self.rc.set_ur5_angles(start_angles, instant=True)
                return None

        except Exception as e:
            print(f"[RobotIK] Exception: {e}")
            self.rc.set_ur5_angles(start_angles, instant=True)
            return None

    def check_reachability(self, pos, quat, ref_frame=Frames.WORLD) -> bool:
        """快速检查某个点是否可达 (不改变机器人当前状态)"""
        current = self.rc.get_ur5_angles()
        res = self.solve(pos, quat, ref_frame, restore_if_fail=True)
        # 恢复状态
        self.rc.set_ur5_angles(current, instant=True)
        return res is not None
