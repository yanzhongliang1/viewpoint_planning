from typing import Optional, Tuple
from scan_qt.test.robot_comm import RobotComm, Frames


class RobotIK:
    """
    基于 simIK 的逆运动学解算器。
    支持全姿态约束和仅位置约束切换。
    """

    def __init__(self, rc: RobotComm):
        self.rc = rc
        self.sim = rc.sim
        self.simIK = rc.simIK

        # 初始化 IK 环境
        self.ik_env = self.simIK.createEnvironment()

        # --- 创建两个 IK 组 ---
        # 1. 全约束 (位置 + 旋转) - 用于正常工作
        self.ik_group_pose = self.simIK.createGroup(self.ik_env)
        self._setup_group(self.ik_group_pose, self.simIK.constraint_pose)

        # 2. 仅位置约束 (忽略旋转) - 用于调试或特殊情况
        self.ik_group_pos = self.simIK.createGroup(self.ik_env)
        self._setup_group(self.ik_group_pos, self.simIK.constraint_position)

        if self.rc.verbose:
            print("[RobotIK] IK Environment initialized (Pose & Position modes).")

    def _setup_group(self, group_handle, constraint_type):
        """辅助函数：配置 IK 链"""
        self.simIK.addElementFromScene(
            self.ik_env,
            group_handle,
            self.rc.handles.base,  # Base
            self.rc.handles.tip,  # Tip
            self.rc.handles.target,  # Target
            constraint_type
        )
        # 设置解算方法 (DLS)
        self.simIK.setGroupCalculation(
            self.ik_env,
            group_handle,
            self.simIK.method_damped_least_squares,
            0.1,
            100
        )

    def solve(self,
              pos: Tuple[float, float, float],
              quat: Tuple[float, float, float, float],
              ref_frame: str = Frames.WORLD,
              restore_if_fail: bool = True,
              ignore_rotation: bool = False) -> Optional[Tuple[float, ...]]:
        """
        执行 IK 解算
        :param ignore_rotation: 如果为 True，则只匹配位置，忽略朝向
        """

        # 1. 记录起点
        start_angles = self.rc.get_ur5_angles()

        try:
            # 2. 设置 Target 位姿
            h_ref = self.rc._resolve_frame(ref_frame)
            self.sim.setObjectPose(self.rc.handles.target, h_ref, list(pos) + list(quat))

            # 3. 选择使用哪个 IK 组
            active_group = self.ik_group_pos if ignore_rotation else self.ik_group_pose

            # 4. 运行解算
            result, _, _ = self.simIK.handleGroup(
                self.ik_env,
                active_group,
                {'syncWorlds': True, 'allowError': True}
            )

            # 1 = Success, 2 = Success with error
            success = (result == 1) or (result == 2)

            if success:
                solved_angles = self.rc.get_ur5_angles()
                # 复位 (为了让物理引擎平滑移动)
                self.rc.set_ur5_angles(start_angles, instant=True)
                return solved_angles
            else:
                if self.rc.verbose:
                    print(f"[RobotIK] Failed. Result code: {result}")
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
