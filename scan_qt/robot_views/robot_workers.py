# scan_qt/robot_views/workers.py
import time
import math
from scan_qt.test.robot_comm import Frames
from PyQt5.QtCore import QThread, pyqtSignal, QMutex


class MonitorThread(QThread):
    # 信号：发送状态字典
    data_signal = pyqtSignal(dict)

    def __init__(self, mutex: QMutex):
        super().__init__()
        self.mutex = mutex  # 引用主窗口的全局锁
        self.running = True
        self.rc = None  # RobotComm 实例引用
        self.recording_traj = False
        self.trajectory_points = []

    def run(self):
        while self.running:
            if not self.rc:
                time.sleep(0.1)
                continue

            try:
                # === 关键：加锁读取数据 ===
                # 只有拿到锁，才能调用 self.rc 的任何方法
                self.mutex.lock()
                try:
                    # 如果开启了同步模式，负责推进一步
                    if self.rc.sync_mode:
                        self.rc.step(wait=False)

                    # 批量获取数据 (Batch Read)
                    joints = self.rc.get_ur5_angles()
                    table = self.rc.get_turntable_angle()
                    sim_time = self.rc.get_sim_time()

                    # 仅在需要时读取 Tip 位置 (比较耗时)
                    if self.recording_traj:
                        # 假设 rc.handles.tip 存在
                        tip_pos = self.rc.sim.getObjectPosition(self.rc.handles.tip, self.rc.handles.world)
                    else:
                        tip_pos = [0, 0, 0]

                finally:
                    # 无论 ZMQ 是否报错，必须解锁，否则主界面死锁
                    self.mutex.unlock()
                # =========================

                # 数据处理放在锁外
                if self.recording_traj:
                    self.trajectory_points.append(tip_pos)

                self.data_signal.emit({
                    "time": sim_time,
                    "joints": joints,
                    "table": table,
                    "tip": tip_pos
                })

            except Exception as e:
                # 忽略瞬时通讯错误，防止线程退出
                # print(f"Monitor Warning: {e}")
                pass

            time.sleep(0.05)  # 20Hz 刷新率

    def clear_trajectory(self):
        self.trajectory_points = []

    def stop(self):
        self.running = False
        # 等待 run() 循环结束，这通常由 wait() 在外部调用完成


import math
import time
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal


class AutoScanWorker(QThread):
    """
    通用扫描工作线程
    支持：单步移动 (viewpoints列表长度为1) 和 全自动扫描 (列表长度>1)
    逻辑：严格复刻 robot_controller.py 的分步执行逻辑
    """
    progress_signal = pyqtSignal(int)  # 发送当前处理的行号
    log_signal = pyqtSignal(str, str)  # 发送日志 (msg, level)
    finished_signal = pyqtSignal()  # 任务结束

    # 常量定义 (参考 robot_controller.py)
    IDEAL_SECTOR_DEG = 45.0
    TIMEOUT = 20.0
    JOINT_TOLERANCE = 0.02

    def __init__(self, rc, ik, path, viewpoints_info, mutex):
        """
        :param viewpoints_info: 列表，每个元素是 tuple (row_index, handle_id)
        """
        super().__init__()
        self.rc = rc
        self.ik = ik
        self.path = path
        self.viewpoints_info = viewpoints_info  # [(row, handle), ...]
        self.mutex = mutex
        self.is_running = True

    def run(self):
        count = len(self.viewpoints_info)
        mode_str = "单步移动" if count == 1 else "全自动扫描"
        self.log_signal.emit(f"=== 开始{mode_str}，共 {count} 个目标 ===", "INFO")

        for i, (row_idx, handle) in enumerate(self.viewpoints_info):
            if not self.is_running: break

            self.progress_signal.emit(row_idx)  # 通知 UI 高亮该行
            self.log_signal.emit(f"正在处理第 {i + 1}/{count} 个视点 (Handle: {handle})...", "INFO")

            try:
                # ========================================================
                # Phase 1: 转台对齐 (Turntable Alignment)
                # ========================================================
                self.log_signal.emit("Phase 1: 调整转台...", "INFO")

                # 1.1 计算目标角度 (加锁获取数据)
                self.mutex.lock()
                try:
                    # 获取各部件世界坐标
                    robot_pos = self.rc.sim.getObjectPosition(self.rc.handles.base, self.rc.handles.world)
                    table_pos = self.rc.sim.getObjectPosition(self.rc.handles.turntable, self.rc.handles.world)
                    vp_pos = self.rc.sim.getObjectPosition(handle, self.rc.handles.world)  # 视点当前位置
                    curr_table_angle = self.rc.get_turntable_angle()
                finally:
                    self.mutex.unlock()

                # 1.2 数学计算 (参考 controller 逻辑)
                base_azimuth = math.atan2(robot_pos[1] - table_pos[1], robot_pos[0] - table_pos[0])
                ideal_azimuth = base_azimuth + math.radians(self.IDEAL_SECTOR_DEG)
                vp_azimuth = math.atan2(vp_pos[1] - table_pos[1], vp_pos[0] - table_pos[0])

                diff = ideal_azimuth - vp_azimuth
                target_table_angle = curr_table_angle + diff

                self.log_signal.emit(f" -> 目标转台角度: {math.degrees(target_table_angle):.1f}°", "INFO")

                # 1.3 发送转台指令
                self.mutex.lock()
                self.rc.set_turntable_angle(target_table_angle, instant=False)
                self.mutex.unlock()

                # 1.4 等待转台物理到位 (关键！)
                if not self._wait_for_table(target_table_angle):
                    self.log_signal.emit(" -> 转台移动超时，跳过。", "WARNING")
                    continue

                # ========================================================
                # Phase 2: 刷新坐标 (Update Pose)
                # ========================================================
                # 转台动了，物体挂在转台上，所以视点的世界坐标变了。
                # 必须重新获取 handle 的坐标。
                self.mutex.lock()
                try:
                    final_pos = self.rc.sim.getObjectPosition(handle, self.rc.handles.world)
                    final_quat = self.rc.sim.getObjectQuaternion(handle, self.rc.handles.world)
                finally:
                    self.mutex.unlock()

                # ========================================================
                # Phase 3: 逆解与移动 (IK & Move)
                # ========================================================
                self.log_signal.emit("Phase 3: IK 解算与机械臂移动...", "INFO")

                # 3.1 解算 (加锁)
                self.mutex.lock()
                # 注意：ref_frame 使用 WORLD (-1)
                solution = self.ik.solve(final_pos, final_quat, ref_frame=-1)
                self.mutex.unlock()

                if solution:
                    # 3.2 发送机械臂指令
                    self.mutex.lock()
                    self.rc.set_ur5_angles(solution, instant=True)
                    self.mutex.unlock()

                    # 3.3 等待机械臂到位
                    if self._wait_for_robot(solution):
                        # 3.4 模拟拍照 (变绿)
                        self.log_signal.emit(" -> ✅ 到位，模拟拍照...", "SUCCESS")
                        self.mutex.lock()
                        self.rc.sim.setObjectColor(handle, 0, self.rc.sim.colorcomponent_ambient_diffuse, [0, 1, 0])
                        self.mutex.unlock()
                        time.sleep(1.0)  # 模拟拍照耗时
                    else:
                        self.log_signal.emit(" -> ⚠️ 机械臂移动超时。", "WARNING")
                else:
                    self.log_signal.emit(" -> ❌ IK 无解 (目标不可达)", "ERROR")
                    self.mutex.lock()
                    self.rc.sim.setObjectColor(handle, 0, self.rc.sim.colorcomponent_ambient_diffuse, [1, 0, 0])
                    self.mutex.unlock()

            except Exception as e:
                self.log_signal.emit(f"执行出错: {e}", "ERROR")
                time.sleep(1)

        self.log_signal.emit(f"=== {mode_str}结束 ===", "INFO")
        self.finished_signal.emit()

    def _wait_for_table(self, target):
        """等待转台到位"""
        start = time.time()
        while time.time() - start < self.TIMEOUT and self.is_running:
            self.mutex.lock()
            curr = self.rc.get_turntable_angle()
            self.mutex.unlock()

            if abs(curr - target) < 0.01:  # 0.5度左右误差
                return True
            time.sleep(0.05)  # 让出时间片
        return False

    def _wait_for_robot(self, target_joints):
        """等待机械臂到位"""
        start = time.time()
        while time.time() - start < self.TIMEOUT and self.is_running:
            self.mutex.lock()
            curr = self.rc.get_ur5_angles()
            self.mutex.unlock()

            diff = max([abs(c - t) for c, t in zip(curr, target_joints)])
            if diff < self.JOINT_TOLERANCE:
                return True
            time.sleep(0.05)
        return False

    def stop(self):
        self.is_running = False