# scan_qt/robot/robot_worker.py
"""
机器人后台工作线程
负责平滑的关节控制、轨迹执行等
"""
import time
import math
import numpy as np
from typing import List, Optional, Callable
from PyQt5.QtCore import QThread, pyqtSignal

from scan_qt.robot.robot_comm import RobotComm
from scan_qt.robot.robot_model import RobotModel


class RobotWorker(QThread):
    """
    机器人后台工作线程
    功能:
    - 平滑关节运动
    - 轨迹执行
    - 实时状态更新
    """

    # 信号
    progress_updated = pyqtSignal(int)  # 进度更新 (0-100)
    status_updated = pyqtSignal(str)  # 状态更新
    motion_finished = pyqtSignal(bool)  # 运动完成 (成功/失败)
    joint_state_updated = pyqtSignal(list, float)  # 关节状态更新

    def __init__(self, comm: RobotComm, model: RobotModel):
        super().__init__()
        self.comm = comm
        self.model = model

        self.is_running = False
        self.should_stop = False

        # 运动参数
        self.dt = 0.01  # 控制周期 10ms
        self.trajectory_points: List[List[float]] = []
        self.current_trajectory_index = 0

    # ==================== 线程控制 ====================

    def run(self):
        """线程主循环"""
        self.is_running = True
        self.should_stop = False

        while self.is_running and not self.should_stop:
            try:
                # 执行轨迹
                if self.trajectory_points:
                    self._execute_trajectory_step()

                # 更新关节状态
                if self.comm.is_connected():
                    ur5_config, turtle_config = self.comm.get_all_joint_positions()
                    self.joint_state_updated.emit(ur5_config, turtle_config)

                time.sleep(self.dt)

            except Exception as e:
                print(f"[RobotWorker] 异常: {e}")
                self.status_updated.emit(f"错误: {e}")
                break

        self.is_running = False

    def stop(self):
        """停止线程"""
        self.should_stop = True
        self.wait()

    # ==================== 运动控制 ====================

    def move_to_config(self, target_ur5: List[float], target_turtle: float,
                       duration: float = 2.0):
        """
        平滑运动到目标配置

        Args:
            target_ur5: 目标 UR5 配置
            target_turtle: 目标转台角度
            duration: 运动时长（秒）
        """
        if not self.comm.is_connected():
            self.status_updated.emit("未连接")
            return

        try:
            # 获取当前配置
            current_ur5, current_turtle = self.comm.get_all_joint_positions()

            # 选择最短路径（处理连续关节）
            target_ur5_adj = self._adjust_target_angles(current_ur5, target_ur5)
            target_turtle_adj = self._adjust_target_angle(current_turtle, target_turtle)

            # 生成轨迹
            num_steps = int(duration / self.dt)
            self.trajectory_points = []

            for i in range(num_steps + 1):
                alpha = i / num_steps
                # 使用平滑插值（S 曲线）
                alpha_smooth = self._smooth_step(alpha)

                config = []
                for c, t in zip(current_ur5, target_ur5_adj):
                    config.append(c + (t - c) * alpha_smooth)

                turtle = current_turtle + (target_turtle_adj - current_turtle) * alpha_smooth

                self.trajectory_points.append(config + [turtle])

            self.current_trajectory_index = 0
            self.status_updated.emit("运动中...")

        except Exception as e:
            self.status_updated.emit(f"运动失败: {e}")
            self.motion_finished.emit(False)

    def _execute_trajectory_step(self):
        """执行轨迹的一步"""
        if self.current_trajectory_index >= len(self.trajectory_points):
            # 轨迹执行完成
            self.trajectory_points.clear()
            self.current_trajectory_index = 0
            self.status_updated.emit("运动完成")
            self.motion_finished.emit(True)
            return

        # 获取当前目标点
        target = self.trajectory_points[self.current_trajectory_index]

        # 下发关节命令
        self.comm.set_all_joint_positions(target[:6], target[6])

        # 更新进度
        progress = int(100 * self.current_trajectory_index / len(self.trajectory_points))
        self.progress_updated.emit(progress)

        self.current_trajectory_index += 1

    def execute_trajectory(self, configs: List[List[float]],
                           duration_per_segment: float = 2.0):
        """
        执行多段轨迹
        Args:
            configs: 配置列表 [[ur5_1..6, turtle], ...]
            duration_per_segment: 每段时长
        """
        if not configs:
            return

        try:
            current_ur5, current_turtle = self.comm.get_all_joint_positions()
            current_config = current_ur5 + [current_turtle]

            self.trajectory_points = []

            for target_config in configs:
                # 调整目标角度
                target_ur5_adj = self._adjust_target_angles(
                    current_config[:6], target_config[:6]
                )
                target_turtle_adj = self._adjust_target_angle(
                    current_config[6], target_config[6]
                )

                # 生成插值点
                num_steps = int(duration_per_segment / self.dt)

                for i in range(1, num_steps + 1):
                    alpha = i / num_steps
                    alpha_smooth = self._smooth_step(alpha)

                    config = []
                    for c, t in zip(current_config[:6], target_ur5_adj):
                        config.append(c + (t - c) * alpha_smooth)

                    turtle = current_config[6] + (target_turtle_adj - current_config[6]) * alpha_smooth

                    self.trajectory_points.append(config + [turtle])

                # 更新当前配置
                current_config = target_ur5_adj + [target_turtle_adj]

            self.current_trajectory_index = 0
            self.status_updated.emit(f"执行轨迹 ({len(configs)} 个点)...")

        except Exception as e:
            self.status_updated.emit(f"轨迹执行失败: {e}")
            self.motion_finished.emit(False)

    # ==================== 工具方法 ====================

    @staticmethod
    def _adjust_target_angles(current: List[float], target: List[float]) -> List[float]:
        """调整目标角度到最近分支"""
        adjusted = []
        for c, t in zip(current, target):
            diff = t - c
            while diff > math.pi:
                t -= 2 * math.pi
                diff = t - c
            while diff < -math.pi:
                t += 2 * math.pi
                diff = t - c
            adjusted.append(t)
        return adjusted

    @staticmethod
    def _adjust_target_angle(current: float, target: float) -> float:
        """调整单个角度到最近分支"""
        diff = target - current
        while diff > math.pi:
            target -= 2 * math.pi
            diff = target - current
        while diff < -math.pi:
            target += 2 * math.pi
            diff = target - current
        return target

    @staticmethod
    def _smooth_step(t: float) -> float:
        """
        平滑插值函数（S 曲线）

        Args:
            t: 参数 [0, 1]

        Returns:
            平滑后的参数 [0, 1]
        """
        # 使用 smoothstep 函数: 3t² - 2t³
        return t * t * (3.0 - 2.0 * t)
