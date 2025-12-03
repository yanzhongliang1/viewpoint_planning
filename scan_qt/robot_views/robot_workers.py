# scan_qt/robot_views/workers.py
import time
import logging
from PyQt5.QtCore import QThread, pyqtSignal, QMutex


class MonitorThread(QThread):
    data_signal = pyqtSignal(dict)

    def __init__(self, robot_comm, mutex):
        super().__init__()
        self.rc = robot_comm
        self.mutex = mutex  # 必须从外部传入互斥锁
        self.running = True
        self.recording_traj = False
        self.trajectory_points = []

    def run(self):
        while self.running:
            if not self.rc:
                time.sleep(0.1)
                continue

            # 使用 try-except 包裹，防止单次通讯失败导致线程崩溃
            try:
                # === 关键点：加锁 ===
                # 保证获取数据时，主线程没有在发送指令
                self.mutex.lock()

                # 如果开启了同步模式，这里负责步进
                if self.rc.sync_mode:
                    self.rc.step(wait=False)

                # 获取数据
                joints = self.rc.get_ur5_angles()
                table = self.rc.get_turntable_angle()
                tip_pos = self.rc.get_handle_position(self.rc.handles.tip)
                sim_time = self.rc.get_sim_time()

                # 释放锁
                self.mutex.unlock()
                # ===================

                if self.recording_traj:
                    self.trajectory_points.append(tip_pos)

                self.data_signal.emit({
                    "time": sim_time,
                    "joints": joints,
                    "table": table,
                    "tip": tip_pos
                })

            except Exception as e:
                # 发生错误时也要确保释放锁，否则死锁
                try:
                    self.mutex.unlock()
                except:
                    pass
                    # logging.debug(f"Monitor error: {e}") # 可选：打印调试信息

            time.sleep(0.05)  # 20Hz 刷新率

    def clear_trajectory(self):
        self.trajectory_points = []

    def stop(self):
        self.running = False
        self.wait()
