# scan_qt\test\robot_controller.py
import sys
import os
import time
import math
import numpy as np



from scan_qt.test.robot_comm import RobotComm, Frames
from scan_qt.test.robot_ik import RobotIK
from scan_qt.test.robot_path import RobotPath

# --- 配置参数 ---
# 强制待机位置 (关节角 rad)
HOME_JOINTS = [0, 0, 0, 0, 0, 0]
# 视点文件路径
VIEWPOINTS_FILE = "D:/Viewpoint Planning/Auto_Scan/scan_qt/scan_qt/resources/viewpoints.txt"
# 理想拍摄扇区 (机器人前方多少度)
IDEAL_SECTOR_DEG = 45.0

class AutoScanner:
    def __init__(self):
        print("=== 初始化扫描系统 ===")
        self.rc = RobotComm(start_sim=True)
        self.ik = RobotIK(self.rc)
        self.path = RobotPath(self.rc)

        # 物理参数
        self.timeout = 20
        self.joint_tolerance = 0.02

    def load_data(self):
        """读取视点文件并创建可视化"""
        if not os.path.exists(VIEWPOINTS_FILE):
            print(f"[Error] 找不到文件: {VIEWPOINTS_FILE}")
            self.rc.stop()
            sys.exit(1)

        print(f"[System] 读取视点: {VIEWPOINTS_FILE}")
        self.path.load_viewpoints_from_txt(VIEWPOINTS_FILE)
        self.path.create_visuals()
        print(f"[System] 共加载 {len(self.path.viewpoints)} 个视点")

    def run(self):
        """主执行循环"""
        try:
            # 1. 系统预热 & 归位
            print("\n[Step0] 机器人归位(HomeCheck)...")
            self.rc.set_ur5_angles(HOME_JOINTS, instant=False)
            time.sleep(2.0)  # 给予物理运动时间

            # 2. 开始循环
            print("\n >> > 开始自动化扫描任务 << < ")

            for i, vp in enumerate(self.path.viewpoints):
                self._process_single_viewpoint(i, vp)
            print("\n>> > 所有任务完成 << < ")

        except KeyboardInterrupt:
            print("\n[System]用户中断任务")
        finally:
            print("[System] 正在停止仿真...")
            time.sleep(2)
            self.rc.stop()

    def _process_single_viewpoint(self, index, vp):
        print(f"\n== == == == == == == == [Viewpoint ID: {vp.id} | {vp.name}] == == == == == == == == ")

        # ------------------------------------------------------
        # Phase 1: 转台对齐 (Turntable Alignment)
        # ------------------------------------------------------
        print("[Phase 1] 转台调整...")

        # 获取基准位置
        robot_pos = self.rc.sim.getObjectPosition(self.rc.handles.base, self.rc.handles.world)
        table_pos = self.rc.sim.getObjectPosition(self.rc.handles.turntable, self.rc.handles.world)

        # 计算当前视点相对于转台的角度 (注意：必须取当前的实时位置)
        # 此时 Dummy 的位置可能还没更新，所以我们通过 RobotPath 算一下
        curr_vp_pos, _ = self.path.get_vp_world_pose(index)

        # 向量角度计算
        base_azimuth = math.atan2(robot_pos[1] - table_pos[1], robot_pos[0] - table_pos[0])
        ideal_azimuth = base_azimuth + math.radians(IDEAL_SECTOR_DEG)
        vp_azimuth = math.atan2(curr_vp_pos[1] - table_pos[1], curr_vp_pos[0] - table_pos[0])

        # 计算偏差并旋转
        diff = ideal_azimuth - vp_azimuth
        curr_table_angle = self.rc.get_turntable_angle()
        target_table_angle = curr_table_angle + diff

        print(f" -> 目标转台角度: {math.degrees(target_table_angle):.1f}° (旋转 {math.degrees(diff):.1f}°)")

        self.rc.set_turntable_angle(target_table_angle, instant=False)

        # 等待转台物理停止
        if not self._wait_for_table(target_table_angle):
            print(" -> [Error] 转台超时，跳过此点。")
            return

        # ------------------------------------------------------
        # Phase 2: 刷新坐标 (Update World Pose)
        # ------------------------------------------------------
        # 核心：转台动了，Receiver动了，必须更新 Dummy 的 World 坐标
        self.path.update_all_dummies_pose()

        # 获取最终真值
        final_pos, final_quat = self.path.get_vp_world_pose(index)
        print(f"[Phase 2] 更新世界坐标:")
        print(f" -> Pos: [{final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f}]")

        # ------------------------------------------------------
        # Phase 3: 逆解与移动 (IK & Move)
        # ------------------------------------------------------
        print("[Phase 3] 逆解与移动...")

        solution = self.ik.solve(final_pos, final_quat, ref_frame=Frames.WORLD)

        if solution:
            target_joints = list(solution)

            # 检查是否需要移动
            curr_joints = self.rc.get_ur5_angles()
            move_diff = max([abs(a - b) for a, b in zip(curr_joints, target_joints)])
            print(f" -> 关节变动量: {move_diff:.4f} rad")

            if move_diff > 0.01:
                print(" -> 🚀 机器人移动中...")
                self.rc.set_ur5_angles(target_joints, instant=False)

                if self._wait_for_robot(target_joints):
                    print(" -> ✅ 到位，模拟拍摄 (Dummy变绿)")
                    self.rc.sim.setObjectColor(vp.handle, 0, self.rc.sim.colorcomponent_ambient_diffuse, [0, 1, 0])
                    time.sleep(2.0)  # 模拟拍照时间
                else:
                    print(" -> ⚠️ 移动超时。")
            else:
                print(" -> 机器人已在位置，直接拍摄。")
                self.rc.sim.setObjectColor(vp.handle, 0, self.rc.sim.colorcomponent_ambient_diffuse, [0, 1, 0])
                time.sleep(1.0)
        else:
            print(" -> ❌ IK 无解 (Dummy变红)")
            self.rc.sim.setObjectColor(vp.handle, 0, self.rc.sim.colorcomponent_ambient_diffuse, [1, 0, 0])

    # --- 辅助等待函数 --
    def _wait_for_table(self, target):
        start = time.time()
        while time.time() - start < self.timeout:
            self.rc.step(wait=True)
            if abs(self.rc.get_turntable_angle() - target) < 0.01:
                return True
        return False

    def _wait_for_robot(self, target_joints):
        start = time.time()
        while time.time() - start < self.timeout:
            self.rc.step(wait=True)
            self.path.update_trail()  # 画轨迹
            curr = self.rc.get_ur5_angles()
            if max([abs(c - t) for c, t in zip(curr, target_joints)]) < self.joint_tolerance:
                return True
        return False
