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
    COMFORT_LIMIT_DEG = 30.0
    IDEAL_OFFSET_DEG = 45

    def __init__(self, rc, ik, path, target_handles, mutex):
        super().__init__()
        self.rc = rc
        self.ik = ik
        self.path = path
        self.target_handles = target_handles  # 这是一个列表，单步时长度为1
        self.mutex = mutex
        self.is_running = True

    """
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
    """

    # 在 SmartScanWorker 类中添加排序函数
    def _optimize_path_order(self, handles):
        """
        借鉴论文的全局规划思想：
        1. 计算每个视点相对于转台中心的“极角”(Azimuth)。
        2. 按角度对视点进行排序，形成一条连续的扫描轨迹（类似 TSP 的贪心解）。
        """
        self.log_signal.emit("正在进行全局路径规划 (TSP Optimization)...", "INFO")

        viewpoint_data = []

        self.mutex.lock()
        try:
            for h in handles:
                # 获取物体在转台坐标系下的位置
                # 注意：如果 Sim 中没有父子关系，可以用世界坐标差值计算
                # 这里假设我们用世界坐标差值，因为转台在转，我们需要它的初始形态
                # 但为了简化，我们直接取当前物体相对于转台中心的位置

                # 获取物体世界坐标
                p_world = self.rc.sim.getObjectPosition(h, self.rc.handles.world)
                # 获取转台世界坐标
                t_world = self.rc.sim.getObjectPosition(self.rc.handles.turntable, self.rc.handles.world)

                # 计算向量 (Target - Turntable)
                dx = p_world[0] - t_world[0]
                dy = p_world[1] - t_world[1]
                dz = p_world[2]  # Z轴高度

                # 计算极角 (0 到 2pi)
                azimuth = math.atan2(dy, dx)
                if azimuth < 0: azimuth += 2 * math.pi

                viewpoint_data.append({
                    'handle': h,
                    'azimuth': azimuth,
                    'height': dz
                })
        finally:
            self.mutex.unlock()

        # 核心排序逻辑：
        # 1. 主要按角度排序 (减少转台往复)
        # 2. 次要按高度排序 (减少机械臂上下大幅挥动)
        viewpoint_data.sort(key=lambda x: (x['azimuth'], x['height']))

        return [item['handle'] for item in viewpoint_data]

    # 修改 run 方法
    def run(self):
        # 1. 如果是全自动扫描（多点），先优化路径
        if len(self.target_handles) > 1:
            self.target_handles = self._optimize_path_order(self.target_handles)

        total = len(self.target_handles)
        self.log_signal.emit(f"=== 开始智能扫描，共 {total} 个目标 ===", "INFO")

        for i, handle in enumerate(self.target_handles):
            if not self.is_running: break
            self.progress_signal.emit(i)  # 注意：这里对应的是排序后的索引，UI上可能需要适配
            self._process_single_viewpoint_robust(handle)  # 使用新的鲁棒方法
            if total > 1: time.sleep(0.2)

        self.log_signal.emit("=== 任务结束 ===", "INFO")
        self.finished_signal.emit()

    def _process_single_viewpoint_robust(self, handle):
        """
        完全依照 robot_controller.py 的向量逻辑进行计算
        """
        try:
            self.log_signal.emit(f"正在规划 Handle: {handle}", "INFO")

            # ==========================================
            # 1. 获取几何信息 (世界坐标系)
            # ==========================================
            self.mutex.lock()
            # 务必先刷新一下 Dummy 的位置，虽然这里主要靠转台和机器人的相对关系
            self.path.update_all_dummies_pose()

            # 获取三个关键点的世界坐标
            robot_pos = self.rc.sim.getObjectPosition(self.rc.handles.base, self.rc.handles.world)
            table_pos = self.rc.sim.getObjectPosition(self.rc.handles.turntable, self.rc.handles.world)
            # 注意：这里取 Dummy 当前的世界坐标
            vp_pos = self.rc.sim.getObjectPosition(handle, self.rc.handles.world)
            self.mutex.unlock()

            # ==========================================
            # 2. 计算向量角度 (完全复刻 robot_controller)
            # ==========================================

            # A. 基准角度：转台 -> 机器人 (Table to Robot)
            # 这是您的 "Robot Azimuth"
            base_azimuth = math.atan2(robot_pos[1] - table_pos[1], robot_pos[0] - table_pos[0])

            # B. 视点角度：转台 -> 视点 (Table to Viewpoint)
            vp_azimuth = math.atan2(vp_pos[1] - table_pos[1], vp_pos[0] - table_pos[0])

            # C. 偏差计算
            # 理想角度 = 机器人角度 + 偏置 (比如正对就是+0)
            ideal_azimuth = base_azimuth + math.radians(self.IDEAL_OFFSET_DEG)

            # 偏差 = 理想 - 当前
            diff_rad = ideal_azimuth - vp_azimuth

            # 归一化到 -pi ~ pi
            diff_rad = (diff_rad + math.pi) % (2 * math.pi) - math.pi
            diff_deg = math.degrees(diff_rad)

            # ==========================================
            # 3. 舒适区判断
            # ==========================================
            in_comfort_zone = abs(diff_deg) <= self.COMFORT_LIMIT_DEG
            ik_solution = None

            if in_comfort_zone:
                self.log_signal.emit(f"✅ 舒适区内 (偏差{diff_deg:.1f}°)，尝试直接逆解...", "INFO")
                ik_solution = self._try_solve_ik(handle)

                if ik_solution is None:
                    self.log_signal.emit("⚠️ 舒适区内IK失败，准备旋转...", "WARNING")
                    in_comfort_zone = False  # 强制去转动

            # ==========================================
            # 4. 转台旋转逻辑
            # ==========================================
            if not in_comfort_zone:
                self.log_signal.emit(f"🔄 需要调整转台 (需旋转 {diff_deg:.1f}°)...", "INFO")

                # 计算目标角度 (参考 robot_controller: target = current + diff)
                self.mutex.lock()
                curr_table_angle = self.rc.get_turntable_angle()
                self.mutex.unlock()

                # 目标 = 当前角度 + 偏差
                # 逻辑：如果 diff 是正的 (理想在当前左边)，转台逆时针转 (+)，把物体带过去
                target_table_angle = curr_table_angle + diff_rad

                # 尝试微调 (Resampling: 正中, +10度, -10度)
                offsets = [0, 10, -10]

                for offset in offsets:
                    final_target = target_table_angle + math.radians(offset)

                    self.log_signal.emit(f"-> 执行旋转: {math.degrees(final_target):.1f}° (偏置{offset})", "INFO")

                    # A. 发送动作
                    self.mutex.lock()
                    self.rc.set_turntable_angle(final_target, instant=False)
                    self.mutex.unlock()

                    # B. 严格等待到位
                    if not self._wait_for_table(final_target):
                        continue  # 超时重试

                    # C. [最关键的一步] 刷新 Dummy 坐标
                    # 转台动了 -> 物体动了 -> IK输入变了
                    self.mutex.lock()
                    self.path.update_all_dummies_pose()
                    self.mutex.unlock()

                    # D. 在新位置算 IK
                    ik_solution = self._try_solve_ik(handle)
                    if ik_solution:
                        self.log_signal.emit("-> 旋转后逆解成功", "INFO")
                        break  # 成功找到解，退出尝试循环

            # ==========================================
            # 5. 机械臂执行
            # ==========================================
            if ik_solution:
                target_joints = list(ik_solution)
                self.mutex.lock()
                self.rc.set_ur5_angles(target_joints, instant=True)
                self.mutex.unlock()

                if self._wait_for_robot(target_joints):
                    self.log_signal.emit("📸 扫描成功", "SUCCESS")
                    self._set_dummy_color(handle, [0, 1, 0])
                    time.sleep(2)
                else:
                    self.log_signal.emit("❌ 机械臂移动超时", "ERROR")
                    self._set_dummy_color(handle, [1, 0, 0])
            else:
                self.log_signal.emit("❌ 无法到达该点", "ERROR")
                self._set_dummy_color(handle, [1, 0, 0])

        except Exception as e:
            self.log_signal.emit(f"流程异常: {e}", "ERROR")
            import traceback
            print(traceback.format_exc())

    def _try_solve_ik(self, handle):
        """只计算逆解，不移动，返回解或 None"""
        self.mutex.lock()
        try:
            # 此时 Dummy 的位置必须已经是更新过的世界坐标
            pos = self.rc.sim.getObjectPosition(handle, self.rc.handles.world)
            quat = self.rc.sim.getObjectQuaternion(handle, self.rc.handles.world)
            solution = self.ik.solve(pos, quat, ref_frame=Frames.WORLD)
            return solution
        except Exception:
            return None
        finally:
            self.mutex.unlock()

    """
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
    """

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


    def _wait_for_table(self, target_angle_rad, tolerance_deg=1.0, timeout=10.0):
        """严格等待转台到位"""
        start_time = time.time()
        while self.is_running:
            # 1. 获取当前角度
            self.mutex.lock()
            current = self.rc.get_turntable_angle()
            self.mutex.unlock()

            # 2. 计算误差 (注意圆周问题)
            diff = abs(current - target_angle_rad)
            # 处理 359度 和 1度 的情况
            if diff > math.pi: diff = 2 * math.pi - diff

            # 3. 判断到位
            if math.degrees(diff) < tolerance_deg:
                # 额外等待一点点时间让物理引擎消抖
                time.sleep(0.2)
                return True

            # 4. 超时判断
            if time.time() - start_time > timeout:
                self.log_signal.emit(
                    f"转台移动超时! 目标: {math.degrees(target_angle_rad):.1f}, 当前: {math.degrees(current):.1f}",
                    "WARNING")
                return False

            time.sleep(0.1)  # 10Hz 轮询
        return False

    def _wait_for_robot(self, target_joints, tolerance=0.01, timeout=10.0):
        """严格等待机械臂到位"""
        start_time = time.time()
        while self.is_running:
            self.mutex.lock()
            current = self.rc.get_ur5_angles()
            self.mutex.unlock()

            # 计算最大关节误差
            max_diff = max([abs(c - t) for c, t in zip(current, target_joints)])

            if max_diff < tolerance:
                time.sleep(0.1)  # 物理消抖
                return True

            if time.time() - start_time > timeout:
                self.log_signal.emit(f"机械臂移动超时! 最大误差: {max_diff:.4f}", "WARNING")
                return False

            time.sleep(0.1)
        return False
