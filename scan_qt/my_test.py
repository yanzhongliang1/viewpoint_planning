import time
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class IKPathTester:
    def __init__(self, robot_base="/UR5", tip_name="tip", target_name="/target", goal_name="/goal"):
        """
        初始化测试类
        :param robot_base: 机器人的基座名称
        :param tip_name: 机器人末端 Dummy 的相对路径或名称
        :param target_name: 目标 Dummy 的名称
        """
        print("=== 初始化 IK Path Tester (Class Version) ===")
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.simIK = self.client.require('simIK')

        # 场景对象名称配置
        self.cfg_base = robot_base
        self.cfg_tip = f"{robot_base}/{tip_name}"  # 假设 tip 在机器人下
        self.cfg_target = target_name
        self.cfg_goal = goal_name

        # 句柄缓存
        self.h_base = -1
        self.h_tip = -1
        self.h_target = -1
        self.h_joints = []
        self.h_goal = -1

        # 状态标志
        self.dyn_model = False
        self.generated_path = []

    def _init_handles(self):
        """获取场景中的句柄"""
        print("-> 获取对象句柄...")
        try:
            self.h_base = self.sim.getObject(self.cfg_base)
            self.h_tip = self.sim.getObject(self.cfg_tip)
            self.h_target = self.sim.getObject(self.cfg_target)
            self.h_goal = self.sim.getObject(self.cfg_goal)

            self.h_joints = []
            for i in range(1, 7):
                j_name = f"{self.cfg_base}/joint{i}"
                self.h_joints.append(self.sim.getObject(j_name))

            # 自动检测动力学模式
            self.dyn_model = self.sim.isDynamicallyEnabled(self.h_joints[0])
            print(f"   [Info] 动力学模式 (Dynamic): {self.dyn_model}")
            print("   [Info] 句柄获取成功。")
            return True

        except Exception as e:
            print(f"   [Error] 获取句柄失败: {e}")
            print(f"   请检查场景中是否存在: {self.cfg_base}, {self.cfg_tip}, {self.cfg_target}")
            return False

    def generate_ik_path(self, steps=300):
        """
        调用 SimIK 生成路径
        对应 Lua: simIK.generatePath
        """
        print(f"-> 计算 IK 路径 ({steps}步)...")

        # 1. 创建环境
        ik_env = self.simIK.createEnvironment()
        ik_group = self.simIK.createGroup(ik_env)

        # 2. 场景映射 (Scene -> IK Environment)
        # 这一步确立了 Start Pose (机器人当前位置) 和 Goal Pose (Target的位置)
        try:
            ik_element, sim_to_ik_map, ik_to_sim_map = self.simIK.addElementFromScene(
                ik_env,
                ik_group,
                self.h_base,
                self.h_tip,
                self.h_target,
                self.simIK.constraint_pose
            )

            # 从 Map 中获取 IK 环境内部的句柄
            ik_tip_handle = sim_to_ik_map[self.h_tip]
            ik_joint_handles = [sim_to_ik_map[h] for h in self.h_joints]

            # 3. 生成路径
            # path 是一个扁平的 list: [j1_0, j2_0... j6_0, j1_1, j2_1...]
            self.generated_path = self.simIK.generatePath(
                ik_env,
                ik_group,
                ik_joint_handles,
                ik_tip_handle,
                steps
            )

            if not self.generated_path:
                print("   [Error] 路径生成返回为空！目标可能不可达。")
                return False

            print(f"   [Success] 路径已生成，包含 {len(self.generated_path) // 6} 个航点。")
            return True

        except Exception as e:
            print(f"   [Error] SimIK 计算出错: {e}")
            return False
        finally:
            # 4. 清理环境
            self.simIK.eraseEnvironment(ik_env)

    def _hop_through_configs(self, reverse=False):
        """
        执行路径运动
        对应 Lua: hopThroughConfigs
        """
        path = self.generated_path
        dof = 6
        num_configs = len(path) // dof

        # 确定遍历顺序
        if not reverse:
            indices = range(0, num_configs, 1)
        else:
            indices = range(num_configs - 1, -1, -1)

        # 执行循环
        for i in indices:
            start_idx = i * dof
            # 切片获取当前步的6个关节角
            current_joints = path[start_idx: start_idx + dof]

            if self.dyn_model:
                # 动力学模式
                for h, val in zip(self.h_joints, current_joints):
                    self.sim.setJointTargetPosition(h, val)
            else:
                # 运动学模式 (瞬移)
                for h, val in zip(self.h_joints, current_joints):
                    self.sim.setJointPosition(h, val)

            # 推进一步仿真
            self.sim.step()

    def run(self):
        """主运行逻辑"""
        # 1. 启动仿真
        self.sim.setStepping(True)
        self.sim.startSimulation()

        try:
            # 2. 初始化
            if not self._init_handles():
                return

            # 3. 生成路径
            # 注意：这里生成的路径是从 [当前位置] -> [Target]
            if not self.generate_ik_path(steps=300):
                return

            print("-> 开始循环执行 (按 Ctrl+C 停止)...")

            # 4. 循环运动
            while True:
                # 去程
                print("   >>> Forward (Start -> Target)")
                self._hop_through_configs(reverse=False)

                # 回程
                print("   <<< Backward (Target -> Start)")
                self._hop_through_configs(reverse=True)

        except KeyboardInterrupt:
            print("[System] 用户手动停止。")
        except Exception as e:
            print(f"[System] 运行时错误: {e}")
        finally:
            self.stop()

    def stop(self):
        """停止仿真"""
        print("-> 停止仿真...")
        self.sim.stopSimulation()


# --- 程序入口 ---
if __name__ == "__main__":
    # 实例化并运行
    # 请确保这里的名字与你 CoppeliaSim 场景中的层级一致
    tester = IKPathTester(
        robot_base="/UR5",
        tip_name="tip",  # 组合后寻找 /UR5/tip
        target_name="/target"
    )
    tester.run()