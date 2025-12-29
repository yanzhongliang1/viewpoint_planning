# scan_qt/robot_views/robot_window_slots.py
import math
import time
import logging
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QTableWidgetItem
from PyQt5.QtCore import QMutex

# 引入 Backend
from scan_qt.test.robot_comm import RobotComm, Frames
from scan_qt.test.robot_ik import RobotIK
from scan_qt.test.robot_path import RobotPath
from scan_qt.robot_views.robot_workers import SmartScanWorker


class RobotWindowSlots:
    """混入类：负责所有业务逻辑"""

    def setup_connections(self):
        # 连接
        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)

        # 运动控制
        self.btn_home.clicked.connect(self.on_home)
        for i, (slider, spin) in enumerate(zip(self.sliders, self.spinboxes)):
            slider.valueChanged.connect(lambda val, idx=i: self.on_slider_change(idx, val))
            spin.valueChanged.connect(lambda val, idx=i: self.on_spin_change(idx, val))

        # 视点与轨迹
        self.chk_show_traj.toggled.connect(self.on_toggle_traj)
        self.btn_clear_traj.clicked.connect(self.on_clear_traj)
        self.btn_export_plot.clicked.connect(self.on_export_matplotlib)
        self.btn_create_manual.clicked.connect(self.on_create_manual_dummy)
        self.btn_load_file.clicked.connect(self.on_load_vp_file)

        # 智能运动
        self.btn_ik_move.clicked.connect(self.on_ik_move_smart)
        # === [新增] 连接新按钮 ===
        self.btn_full_scan.clicked.connect(self.on_start_full_scan)
        self.btn_stop_scan.clicked.connect(self.on_stop_full_scan)

        # UI 显隐
        self.chk_show_plot.toggled.connect(lambda x: self.plot_widget.setVisible(x))

    # --- 1. 稳健连接与断开 ---
    def on_connect(self):
        host = self.edit_host.text()
        port = int(self.edit_port.text())

        # 在连接期间加锁，防止其他操作干扰初始化
        self.zmq_mutex.lock()
        try:
            logging.info(f"Connecting to {host}:{port}...")

            # 初始化 Backend
            self.rc = RobotComm(host=host, port=port, start_sim=True)
            self.ik = RobotIK(self.rc)
            self.path = RobotPath(self.rc)

            # 配置线程并启动
            self.monitor_thread.rc = self.rc
            if not self.monitor_thread.isRunning():
                self.monitor_thread.start()

            # 初始化轨迹状态
            self.path.init_trail()
            self.monitor_thread.recording_traj = True

            # UI 更新
            self.status_label.setText(f"Connected: {host}")
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.edit_host.setEnabled(False)
            self.edit_port.setEnabled(False)
            logging.info("系统连接成功 (System Online)")

        except Exception as e:
            logging.error(f"连接失败: {e}")
            QMessageBox.critical(self, "连接错误", f"无法连接到 CoppeliaSim。{e}")
            self.rc = None
        finally:
            self.zmq_mutex.unlock()

    # --- 1. 稳健连接与断开 ---
    def on_disconnect(self):
        logging.info("正在断开连接...")

        # 1. 第一步：停止监控线程
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.stop()
            self.monitor_thread.wait()  # 等待线程完全退出

        # 2. 第二步：加锁清理资源
        self.zmq_mutex.lock()
        try:
            if self.rc:
                # === [新增需求] 清除所有轨迹和 Dummy ===
                try:
                    logging.info("清理场景对象...")
                    if self.path:
                        self.path.clear_all()  # 清除 Dummy 和 3D轨迹线
                    if hasattr(self.monitor_thread, 'clear_trajectory'):
                        self.monitor_thread.clear_trajectory()  # 清除内存数据
                except Exception as e:
                    logging.warning(f"清理场景失败: {e}")

                # === [关键修复] 解决重连报错 ===
                # 不要强行调用 sim.stopSimulation() 后立即销毁对象。
                # ZMQ Server 需要一点时间来处理停止命令。
                try:
                    # 只有在仿真还在运行时才停止
                    if self.rc.sim.getSimulationState() != self.rc.sim.simulation_stopped:
                        self.rc.sim.stopSimulation()
                        # 给一点点时间让指令发送出去，防止 socket 突然断开导致 server 报错
                        time.sleep(0.2)
                except:
                    pass

                # 显式删除引用，触发 Python ZMQ 的析构
                del self.rc

        except Exception as e:
            logging.error(f"清理资源时出错: {e}")
        finally:
            self.rc = None  # 指针置空
            self.zmq_mutex.unlock()

        # 3. 恢复 UI 状态
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.edit_host.setEnabled(True)
        self.edit_port.setEnabled(True)
        self.status_label.setText("已断开 / Disconnected")
        logging.info("连接已安全断开")

    # --- 2. 机器人控制 (全部加锁) ---
    def on_home(self):
        if not self.rc: return
        self.zmq_mutex.lock()
        try:
            logging.info("执行归位 (Homing)...")
            self.rc.set_ur5_angles((0, 0, 0, 0, 0, 0))
            self.rc.set_turntable_angle(0.0)
        finally:
            self.zmq_mutex.unlock()

    def _send_joint_command(self, idx, deg):
        if not self.rc or self.programmatic_update: return

        # 使用 tryLock，如果后台正在忙（例如 IK 解算），则跳过此次滑条更新，避免卡顿
        if self.zmq_mutex.tryLock(20):
            try:
                rad = math.radians(deg)
                if idx < 6:
                    curr = list(self.rc.get_ur5_angles())
                    curr[idx] = rad
                    self.rc.set_ur5_angles(tuple(curr))
                else:
                    self.rc.set_turntable_angle(rad)
            except Exception:
                pass
            finally:
                self.zmq_mutex.unlock()

    def on_slider_change(self, idx, val):
        deg = val / 100.0
        self.spinboxes[idx].blockSignals(True)
        self.spinboxes[idx].setValue(deg)
        self.spinboxes[idx].blockSignals(False)
        self._send_joint_command(idx, deg)

    def on_spin_change(self, idx, val):
        self.sliders[idx].blockSignals(True)
        self.sliders[idx].setValue(int(val * 100))
        self.sliders[idx].blockSignals(False)
        self._send_joint_command(idx, val)

    # --- 3. 数据刷新 (含滤波) ---
    def update_data(self, data):
        """处理来自 Worker 的数据信号"""
        # 1. 获取原始数据
        # data['joints'] 是 tuple, data['table'] 是 float
        raw_joints = list(data['joints']) + [data['table']]
        raw_deg = [math.degrees(x) for x in raw_joints]

        # 2. 初始化滤波状态 (如果是第一次运行)
        if not hasattr(self, 'filtered_joints'):
            self.filtered_joints = raw_deg

        # 3. 执行滤波 (EWMA 算法)
        alpha = 0.2  # 滤波系数 (0.1~0.3)，越小越平滑，延迟越高

        # 创建一个临时列表来存储当前帧的滤波结果
        current_filtered_deg = []

        for i in range(7):
            # 公式: new = alpha * raw + (1-alpha) * old
            filtered_val = alpha * raw_deg[i] + (1 - alpha) * self.filtered_joints[i]

            # 死区控制 (Deadzone): 如果变化极其微小(<0.02度)，则保持原值，彻底消除静止时的抖动
            if abs(filtered_val - self.filtered_joints[i]) < 0.01:
                filtered_val = self.filtered_joints[i]

            # 更新状态
            self.filtered_joints[i] = filtered_val
            current_filtered_deg.append(filtered_val)

        # === 关键修正：定义 filtered_deg 变量，供后续使用 ===
        filtered_deg = current_filtered_deg

        # 4. 更新波形图 (使用滤波后的数据)
        self.plot_data = np.roll(self.plot_data, -1, axis=1)
        for i in range(7):
            self.plot_data[i, -1] = filtered_deg[i]
            self.curves[i].setData(self.plot_data[i])

        # 5. 更新滑条 (若用户未拖动)
        # 这里的 filtered_deg[i] 现在肯定已经定义了，不会报错
        if not any(s.isSliderDown() for s in self.sliders):
            self.programmatic_update = True
            for i in range(7):
                val = filtered_deg[i]
                # 更新 Slider (整数)
                self.sliders[i].setValue(int(val * 100))
                # 更新 SpinBox (浮点数)
                self.spinboxes[i].setValue(val)
            self.programmatic_update = False

        # 6. 刷新轨迹 (需要短暂锁)
        if self.chk_show_traj.isChecked():
            if self.zmq_mutex.tryLock(5):
                try:
                    self.path.update_trail()
                except:
                    pass
                finally:
                    self.zmq_mutex.unlock()

    # --- 4. 智能运动 (AutoScanner Logic) ---
    def on_ik_move_smart(self):
        """单步移动：复刻 robot_controller.py 逻辑，通过 Worker 执行"""
        if not self.rc: return

        # 获取选中的行
        sel = self.vp_table.selectedItems()
        if not sel:
            QMessageBox.information(self, "提示", "请选择一个视点。")
            return

        row = sel[0].row()
        handle = int(self.vp_table.item(row, 0).text())

        # 这里的 handle_list 只包含一个元素
        handle_list = [handle]

        self.start_scan_worker(handle_list)

    def on_start_full_scan(self):
        """全自动扫描：处理所有列表中的点"""
        if not self.rc: return

        row_count = self.vp_table.rowCount()
        if row_count == 0:
            QMessageBox.information(self, "提示", "列表为空。")
            return

        handle_list = []
        for row in range(row_count):
            item = self.vp_table.item(row, 0)
            handle_list.append(int(item.text()))

        self.start_scan_worker(handle_list)

    # 3. 统一的 Worker 启动函数 (新增)
    def start_scan_worker(self, handle_list):
        # 界面互斥：禁止重复点击
        self.btn_ik_move.setEnabled(False)
        self.btn_full_scan.setEnabled(False)
        self.btn_stop_scan.setEnabled(True)
        self.vp_table.setEnabled(False)

        # 实例化 Worker
        self.scan_worker = SmartScanWorker(
            self.rc,
            self.ik,
            self.path,
            handle_list,
            self.zmq_mutex
        )

        # 连接信号
        self.scan_worker.progress_signal.connect(self.on_scan_progress)  # 复用之前的进度高亮
        self.scan_worker.log_signal.connect(self.on_scan_log)  # 复用之前的日志
        self.scan_worker.finished_signal.connect(self.on_scan_finished)  # 复用之前的结束清理

        self.scan_worker.start()

    # 4. 停止按钮 (保持不变)
    def on_stop_full_scan(self):
        if hasattr(self, 'scan_worker') and self.scan_worker.isRunning():
            logging.warning("正在停止任务...")
            self.scan_worker.stop()

    # --- 辅助功能 ---
    def on_create_manual_dummy(self):
        try:
            val = self.input_vp_pos.text()
            parts = [float(x) for x in val.split(',')]

            self.zmq_mutex.lock()
            try:
                h = self.rc.sim.createDummy(0.05)
                self.rc.sim.setObjectPosition(h, -1, parts)
                self.rc.sim.setObjectAlias(h, "Manual_Pt")

                row = self.vp_table.rowCount()
                self.vp_table.insertRow(row)
                self.vp_table.setItem(row, 0, QTableWidgetItem(str(h)))
                self.vp_table.setItem(row, 1, QTableWidgetItem("Manual"))
                self.vp_table.setItem(row, 2, QTableWidgetItem(str(parts)))
            finally:
                self.zmq_mutex.unlock()
        except:
            pass

    def on_load_vp_file(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Load Viewpoints", "", "Text Files (*.txt)")
        if not fname: return

        self.zmq_mutex.lock()
        try:
            self.path.load_viewpoints_from_txt(fname)
            self.path.create_visuals()

            self.vp_table.setRowCount(0)
            for vp in self.path.viewpoints:
                row = self.vp_table.rowCount()
                self.vp_table.insertRow(row)
                self.vp_table.setItem(row, 0, QTableWidgetItem(str(vp.handle)))
                self.vp_table.setItem(row, 1, QTableWidgetItem(vp.name))
                self.vp_table.setItem(row, 2, QTableWidgetItem("Loaded"))
        except Exception as e:
            logging.error(str(e))
        finally:
            self.zmq_mutex.unlock()

    def on_toggle_traj(self, checked):
        self.monitor_thread.recording_traj = checked
        if not checked:
            self.on_clear_traj()

    def on_clear_traj(self):
        self.monitor_thread.clear_trajectory()
        if self.rc:
            self.zmq_mutex.lock()
            try:
                self.path.clear_trail()
            finally:
                self.zmq_mutex.unlock()

    def on_export_matplotlib(self):
        # 简单导出示例
        pts = list(self.monitor_thread.trajectory_points)
        if not pts: return
        data = np.array(pts)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(data[:, 0], data[:, 1], data[:, 2])
        plt.show()

    def close_app_cleanup(self):
        """窗口关闭时的清理"""
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.stop()
            self.monitor_thread.wait()

        self.zmq_mutex.lock()
        try:
            if hasattr(self, 'rc') and self.rc:
                self.rc.stop()
        except:
            pass
        finally:
            self.zmq_mutex.unlock()

    # --- Worker 回调函数 ---
    def on_scan_progress(self, row_index):
        """Worker 通知进度，高亮表格"""
        self.vp_table.selectRow(row_index)
        self.vp_table.scrollToItem(self.vp_table.item(row_index, 0))

    def on_scan_log(self, msg, level):
        """Worker 发送日志"""
        if level == "SUCCESS":
            logging.info(f"✅ {msg}")
        elif level == "WARNING":
            logging.warning(msg)
        elif level == "ERROR":
            logging.error(msg)
        else:
            logging.info(msg)

    def on_scan_finished(self):
        """任务结束恢复 UI"""
        self.btn_ik_move.setEnabled(True)
        self.btn_full_scan.setEnabled(True)
        self.btn_stop_scan.setEnabled(False)
        self.vp_table.setEnabled(True)
        logging.info("任务线程已退出。")