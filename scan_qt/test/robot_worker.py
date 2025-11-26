import time
import math
import numpy as np
from scan_qt.test.robot_comm import Frames


class RobotWorker:
    def __init__(self, rc, ik, path):
        self.rc = rc
        self.sim = rc.sim
        self.ik = ik
        self.path = path

        # --- 1. 安全位置 (Safe Home) ---
        # 这是一个比较直立且收缩的姿态，避免扫到转台
        self.home_joints = [0, -1.57, 0, -1.57, 0, 0]

        # --- 2. 策略配置: 舒适区偏置角 ---
        # 0度 = 视点正对机器人基座 (导致机器人要在 Y=0 平面工作，非常别扭)
        # 45度(0.78rad) = 视点偏向一侧，机器人可以"侧身"去够，形成自然的肘部弯曲
        # 建议尝试: math.radians(30) 到 math.radians(60)
        # 正负号决定了是偏左还是偏右，取决于你的机器人安装习惯
        self.approach_offset = math.radians(45)

        self.h_table_center = self.rc.handles.turntable

    def run_scanning_task(self):
        if not self.path.viewpoints:
            print("[Worker] No viewpoints loaded.")
            return

        self.path.init_trail()

        # --- A. 获取全局几何关系 ---
        robot_pos = self.sim.getObjectPosition(self.rc.handles.base, self.rc.handles.world)
        table_pos = self.sim.getObjectPosition(self.h_table_center, self.rc.handles.world)

        # 计算 "机器人-转台" 的连线角度 (基准角度)
        base_line_azimuth = math.atan2(robot_pos[1] - table_pos[1], robot_pos[0] - table_pos[0])
        print(f"[Worker] Base Line Azimuth: {math.degrees(base_line_azimuth):.1f} deg")

        # 目标方位角 = 基准 + 偏置 (这是我们希望视点最终停在相对于转台的角度)
        ideal_vp_azimuth = base_line_azimuth + self.approach_offset
        print(
            f"[Worker] Strategy: Rotate points to Azimuth {math.degrees(ideal_vp_azimuth):.1f} deg (Offset {math.degrees(self.approach_offset):.1f})")

        for i, vp in enumerate(self.path.viewpoints):
            print(f"\n== = Processing Viewpoint{i}: {vp.name} == = ")

            # 1. 安全回缩
            print(" -> Retracting to Safe Position...")
            self._move_robot_joints(self.home_joints)

            # 2. 计算转台旋转量
            # 获取当前视点在世界坐标系的位置
            curr_vp_pos, _ = self.path.get_vp_world_pose(i)
            # 计算该视点当前相对于转台中心的角度
            vp_current_azimuth = math.atan2(curr_vp_pos[1] - table_pos[1], curr_vp_pos[0] - table_pos[0])

            # 计算差异：我们需要把 vp_current_azimuth 变成 ideal_vp_azimuth
            diff = ideal_vp_azimuth - vp_current_azimuth

            current_table_angle = self.rc.get_turntable_angle()
            target_table_angle = current_table_angle + diff

            print(
                f" -> Rotating Table: {math.degrees(current_table_angle):.1f} -> {math.degrees(target_table_angle):.1f}")

            # 3. 执行转台旋转
            self._rotate_turntable_smooth(target_table_angle)

            # 4. IK 解算
            target_pos, target_quat = self.path.get_vp_world_pose(i)

            # --- Debug: 打印目标点位置，确认是否合理 ---
            dist_to_robot = np.linalg.norm(np.array(target_pos) - np.array(robot_pos))
            print(f"    [Debug] Target World Pos: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
            print(f"    [Debug] Distance to Robot Base: {dist_to_robot:.3f} m")

            if dist_to_robot > 0.85:
                print("    [WARNING] Target might be too far!")
            elif dist_to_robot < 0.3:
                print("    [WARNING] Target might be too close (Self-collision risk)!")

            joints = self.ik.solve(
                target_pos, target_quat,
                ref_frame=Frames.WORLD,
                ignore_rotation=False
            )

            if joints:
                # --- Debug: 检查解算出的角度变化量 ---
                curr_joints = self.rc.get_ur5_angles()
                j_diff = sum([abs(a - b) for a, b in zip(joints, curr_joints)])
                print(f"    [Debug] IK Solution Found. Total Joint Change: {j_diff:.4f} rad")

                if j_diff < 0.01:
                    print("    [WARNING] Solution is same as current pose! Robot won't move.")

                print(" -> Moving Robot...")
                self.sim.setObjectColor(vp.handle, 0, self.sim.colorcomponent_ambient_diffuse, [0, 1, 0])

                success = self._move_robot_joints(joints)

                if success:
                    print(" -> REACHED. Scanning...")
                    self.sim.setObjectColor(vp.handle, 0, self.sim.colorcomponent_ambient_diffuse, [0.5, 0.5, 0.5])
                else:
                    print(" -> Motion Timeout (Robot stuck or too slow).")
            else:
                print(" -> IK Failed! (Position reachable but orientation/joints restricted)")
                self.sim.setObjectColor(vp.handle, 0, self.sim.colorcomponent_ambient_diffuse, [1, 0, 0])  # Red

            time.sleep(0.5)

    def _rotate_turntable_smooth(self, target_angle):
        start_angle = self.rc.get_turntable_angle()
        steps = 40
        for s in range(steps):
            t = (s + 1) / steps
            interp_angle = start_angle + (target_angle - start_angle) * t
            self.rc.set_turntable_angle(interp_angle)
            self.rc.step()
            self.path.update_all_dummies_pose()

    def _move_robot_joints(self, target_joints):
        """
        移动机器人并返回是否成功
        """
        if not target_joints: return False

        # 1. 发送控制指令
        self.rc.set_ur5_angles(target_joints, instant=False)

        # 2. 等待到位
        start_time = time.time()
        timeout = 10  # 给予更多时间
        arrived = False

        while time.time() - start_time < timeout:
            self.rc.step()  # 物理步进
            self.path.update_trail()  # 画轨迹

            # 检查误差
            curr = self.rc.get_ur5_angles()
            # 找出最大单轴误差
            err = max([abs(a - b) for a, b in zip(curr, target_joints)])

            # 打印进度调试 (每0.5秒打印一次，避免刷屏)
            # if (time.time() * 10) % 5 == 0:
            #    print(f"       dist_err: {err:.4f}")

            if err < 0.02:  # 误差小于 0.02弧度 (~1度)
                arrived = True
                break

        if not arrived:
            # 打印最终误差，帮助判断是卡住了还是走得慢
            curr = self.rc.get_ur5_angles()
            err = max([abs(a - b) for a, b in zip(curr, target_joints)])
            print(f"    [Debug] Motion timeout. Final Max Joint Error: {err:.4f} rad")

        return arrived

    def _check_collision_for_config(self, joints):
        return False
