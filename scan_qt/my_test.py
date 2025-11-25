import math
import time
from scan_qt.test.robot_comm import RobotComm, Frames
from scan_qt.test.robot_ik import RobotIK


def main():
    # 1. 启动通信
    rc = RobotComm(start_sim=True, verbose=True)

    # 2. 初始化 IK
    ik = RobotIK(rc)

    try:
        print("\n=== IK Test Start ===")

        # -------------------------------------------------
        # 场景：让扫描仪(Tip)移动到转台上方并在看向中心
        # -------------------------------------------------

        # 目标定义 (相对于工件坐标系 / Turntable)
        # 位置: X=0.3m, Y=0.0m, Z=0.3m (在转台侧上方)
        target_pos = (0.3, 0.0, 0.3)

        # 姿态: 这是一个难点。
        # 我们希望扫描仪朝向 (-X) 方向看 (假设扫描仪Z轴是视线)
        # 这里简单设置一个四元数，代表某种朝向 (例如绕Y轴转90度)
        # 实际中 quaternion 通常由数学库(scipy/numpy)生成
        # 这里使用一个硬编码的四元数测试 (假设是垂直向下或侧视)
        # Q = [0, 0.707, 0, 0.707] 约等于绕Y轴转90度
        target_quat = (0.0, 0.7071, 0.0, 0.7071)

        print(f"Target Pos: {target_pos}")
        print("Solving IK...")

        # 执行解算
        # ref_frame=Frames.TURNTABLE 表示目标是相对于转台的
        joint_solutions = ik.solve(target_pos, target_quat, ref_frame=Frames.TURNTABLE)

        if joint_solutions:
            print("IK Solution Found!")
            print(f"Angles: {[math.degrees(a) for a in joint_solutions]}")

            # -------------------------------------------------
            # 应用运动
            # -------------------------------------------------

            # 方案 A: 瞬移 (直接验证结果)
            # rc.set_ur5_angles(joint_solutions, instant=True)

            # 方案 B: 物理运动 (平滑移动过去)
            print("Executing motion (Dynamic)...")
            rc.set_ur5_angles(joint_solutions, instant=False)

            # 等待运动完成
            for _ in range(100):
                rc.step()

            # 验证误差
            curr_pos, _ = rc.get_pose(Frames.SCANNER, Frames.TURNTABLE)
            dist_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(curr_pos, target_pos)))
            print(f"Final Position Error: {dist_err * 1000:.2f} mm")

        else:
            print("!!! IK Failed to find a solution.")

    except Exception as e:
        print(f"Test Error: {e}")
    finally:
        rc.stop()
        rc.close()


if __name__ == "__main__":
    main()
