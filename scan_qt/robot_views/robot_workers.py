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


from PyQt5.QtCore import QThread, pyqtSignal
import time
import math
import logging
from scan_qt.test.robot_comm import Frames


class SmartScanWorker(QThread):
    """
    智能扫描工作线程
    严格复刻 robot_controller.py 的 AutoScanner 逻辑
    支持单点模式和多点列表模式
    """
    # 信号定义
    progress_signal = pyqtSignal(int)  # 当前处理到第几个
    log_signal = pyqtSignal(str, str)  # 日志 (msg, level)
    finished_signal = pyqtSignal()  # 全部完成

    # 物理参数
    IDEAL_SECTOR_DEG = 45.0
    TIMEOUT = 20.0
    JOINT_TOLERANCE = 0.02

    def __init__(self, rc, ik, path, target_handles, mutex):
        super().__init__()
        self.rc = rc
        self.ik = ik
        self.path = path
        self.target_handles = target_handles  # 这是一个列表，单步时长度为1
        self.mutex = mutex
        self.is_running = True

    def run(self):
        total = len(self.target_handles)
        self.log_signal.emit(f"=== 开始任务，共 {total} 个目标 ===", "INFO")

        for i, handle in enumerate(self.target_handles):
            if not self.is_running: break

            # 通知 UI 选中对应行 (这里假设 handle 是顺序的，或者你可以传 index 进来，简单起见传 i)
            self.progress_signal.emit(i)

            # 执行核心单点逻辑
            self._process_single_viewpoint(handle, i)

            # 如果是多点扫描，稍微停顿
            if total > 1: time.sleep(0.5)

        self.log_signal.emit("=== 任务结束 ===", "INFO")
        self.finished_signal.emit()

    def _process_single_viewpoint(self, handle, index):
        """核心逻辑：完全复刻 AutoScanner._process_single_viewpoint"""
        try:
            self.log_signal.emit(f"正在处理 Handle: {handle}", "INFO")

            # ======================================================
            # Phase 1: 转台对齐 (Turntable Alignment)
            # ======================================================
            self.mutex.lock()
            try:
                # 1.1 获取基准位置
                robot_pos = self.rc.sim.getObjectPosition(self.rc.handles.base, self.rc.handles.world)
                table_pos = self.rc.sim.getObjectPosition(self.rc.handles.turntable, self.rc.handles.world)

                # 1.2 获取 Dummy 当前位置 (尚未对齐前)
                # 注意：为了计算准确，这里直接从 sim 获取一次最新坐标
                curr_vp_pos = self.rc.sim.getObjectPosition(handle, self.rc.handles.world)

                # 1.3 向量角度计算
                base_azimuth = math.atan2(robot_pos[1] - table_pos[1], robot_pos[0] - table_pos[0])
                ideal_azimuth = base_azimuth + math.radians(self.IDEAL_SECTOR_DEG)
                vp_azimuth = math.atan2(curr_vp_pos[1] - table_pos[1], curr_vp_pos[0] - table_pos[0])

                # 1.4 计算偏差
                diff = ideal_azimuth - vp_azimuth
                curr_table_angle = self.rc.get_turntable_angle()
                target_table_angle = curr_table_angle + diff

                self.log_signal.emit(f"-> 目标转台角度: {math.degrees(target_table_angle):.1f}°", "INFO")

                # 1.5 执行转台旋转
                self.rc.set_turntable_angle(target_table_angle, instant=False)
            finally:
                self.mutex.unlock()

            # 1.6 等待转台物理停止 (如果不加锁读数据，可能会跟 MonitorThread 冲突，建议简单加锁)
            if not self._wait_for_table(target_table_angle):
                self.log_signal.emit("-> [Error] 转台超时，跳过此点。", "ERROR")
                return

            # ======================================================
            # Phase 2: 刷新坐标 (Update World Pose)
            # ======================================================
            self.mutex.lock()
            try:
                # 关键：转台动了，Dummy 的世界坐标变了，必须刷新 Path 类内部的缓存
                # 或者直接获取该 Handle 的最新坐标用于 IK
                self.path.update_all_dummies_pose()  # 这一步会更新 path.viewpoints 里的数据

                # 获取刷新后的坐标
                final_pos = self.rc.sim.getObjectPosition(handle, self.rc.handles.world)
                final_quat = self.rc.sim.getObjectQuaternion(handle, self.rc.handles.world)

                self.log_signal.emit(f"-> 更新后坐标: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, ...]", "INFO")
            finally:
                self.mutex.unlock()

            # ======================================================
            # Phase 3: 逆解与移动 (IK & Move)
            # ======================================================
            self.mutex.lock()
            solution = None
            try:
                # 3.1 逆解
                solution = self.ik.solve(final_pos, final_quat, ref_frame=Frames.WORLD)
            except Exception as e:
                self.log_signal.emit(f"IK计算出错: {e}", "ERROR")
            finally:
                self.mutex.unlock()

            if solution:
                target_joints = list(solution)

                # 3.2 检查是否需要移动
                self.mutex.lock()
                curr_joints = self.rc.get_ur5_angles()
                self.mutex.unlock()

                move_diff = max([abs(a - b) for a, b in zip(curr_joints, target_joints)])

                if move_diff > 0.01:
                    self.log_signal.emit("-> 机器人移动中...", "INFO")

                    self.mutex.lock()
                    self.rc.set_ur5_angles(target_joints, instant=False)
                    self.mutex.unlock()

                    if self._wait_for_robot(target_joints):
                        self.log_signal.emit("-> 到位，模拟拍摄 (Dummy变绿)", "SUCCESS")
                        self._set_dummy_color(handle, [0, 1, 0])
                        time.sleep(1.0)  # 模拟拍照
                    else:
                        self.log_signal.emit("-> ⚠️ 移动超时。", "WARNING")
                else:
                    self.log_signal.emit("-> 机器人已在位置，直接拍摄。", "SUCCESS")
                    self._set_dummy_color(handle, [0, 1, 0])
                    time.sleep(0.5)
            else:
                self.log_signal.emit("-> ❌ IK 无解 (Dummy变红)", "ERROR")
                self._set_dummy_color(handle, [1, 0, 0])

        except Exception as e:
            self.log_signal.emit(f"处理视点时发生未捕获异常: {e}", "ERROR")

    # --- 辅助函数 ---
    def _wait_for_table(self, target):
        start = time.time()
        while time.time() - start < self.TIMEOUT and self.is_running:
            self.mutex.lock()
            try:
                curr = self.rc.get_turntable_angle()
            finally:
                self.mutex.unlock()

            if abs(curr - target) < 0.01:
                return True
            time.sleep(0.1)
        return False

    def _wait_for_robot(self, target_joints):
        start = time.time()
        while time.time() - start < self.TIMEOUT and self.is_running:
            self.mutex.lock()
            try:
                curr = self.rc.get_ur5_angles()
            finally:
                self.mutex.unlock()

            if max([abs(c - t) for c, t in zip(curr, target_joints)]) < self.JOINT_TOLERANCE:
                return True
            time.sleep(0.1)
        return False

    def _set_dummy_color(self, handle, rgb):
        self.mutex.lock()
        try:
            self.rc.sim.setObjectColor(handle, 0, self.rc.sim.colorcomponent_ambient_diffuse, rgb)
        except:
            pass
        finally:
            self.mutex.unlock()

    def stop(self):
        self.is_running = False