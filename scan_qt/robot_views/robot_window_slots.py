# scan_qt/robot_views/robot_window_slots.py
import math
import logging
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QTableWidgetItem, QColorDialog
from PyQt5.QtCore import QMutex

from scan_qt.test.robot_comm import RobotComm, Frames
from scan_qt.test.robot_ik import RobotIK
from scan_qt.test.robot_path import RobotPath


class RobotWindowSlots:
    def setup_connections(self):
        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)
        self.btn_home.clicked.connect(self.on_home)

        # 联动控制
        for i, (slider, spin) in enumerate(zip(self.sliders, self.spinboxes)):
            slider.valueChanged.connect(lambda val, idx=i: self.on_slider_change(idx, val))
            spin.valueChanged.connect(lambda val, idx=i: self.on_spin_change(idx, val))

        # 轨迹与视点
        self.chk_show_traj.toggled.connect(self.on_toggle_traj)
        self.btn_export_plot.clicked.connect(self.on_export_matplotlib)
        self.btn_clear_traj.clicked.connect(self.on_clear_traj)
        self.btn_create_manual.clicked.connect(self.on_create_manual_dummy)
        self.btn_load_file.clicked.connect(self.on_load_vp_file)
        self.btn_ik_move.clicked.connect(self.on_ik_move_to_selected)

        # UI 显隐
        self.chk_show_plot.toggled.connect(lambda x: self.dock_plot.setVisible(x))

    # --- 连接 ---
    def on_connect(self):
        host = self.edit_host.text()
        port = int(self.edit_port.text())

        try:
            # 加锁进行初始化，防止冲突
            self.zmq_mutex.lock()

            logging.info(f"Connecting to {host}:{port}...")
            # 1. 实例化
            self.rc = RobotComm(host=host, port=port, start_sim=True)
            self.ik = RobotIK(self.rc)
            self.path = RobotPath(self.rc)

            # 2. 配置监控线程
            self.monitor_thread.rc = self.rc
            self.monitor_thread.start()

            # 3. 初始化状态
            self.path.init_trail()
            self.monitor_thread.recording_traj = True

            self.zmq_mutex.unlock()

            # UI 更新
            self.status_label.setText(f"已连接: {host}")
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.edit_host.setEnabled(False)
            self.edit_port.setEnabled(False)
            logging.info("系统连接成功")

        except Exception as e:
            self.zmq_mutex.unlock()  # 确保解锁
            logging.error(f"连接失败: {e}")
            QMessageBox.critical(self, "连接错误", f"无法连接到仿真软件。{e}")

    def on_disconnect(self):
        self.monitor_thread.stop()

        self.zmq_mutex.lock()
        if self.rc:
            try:
                self.rc.stop()
            except:
                pass
            self.rc = None
        self.zmq_mutex.unlock()

        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.edit_host.setEnabled(True)
        self.edit_port.setEnabled(True)
        self.status_label.setText("已断开")

    # --- 机器人控制 (加锁保护) ---
    def on_home(self):
        if not self.rc: return
        self.zmq_mutex.lock()
        try:
            self.rc.set_ur5_angles((0,) * 6)
            self.rc.set_turntable_angle(0)
        finally:
            self.zmq_mutex.unlock()

    def _send_joint_command(self, idx, deg):
        if not self.rc or self.programmatic_update: return

        # 尝试获取锁，如果获取不到(正在读取数据)，则跳过此次发送，防止界面卡顿
        if self.zmq_mutex.tryLock(10):
            try:
                rad = math.radians(deg)
                if idx < 6:
                    # 获取当前所有关节（注意：这里也会调用ZMQ）
                    # 为了安全，建议 Monitor 线程缓存一份 joints 到 self.current_joints
                    # 但这里简化处理，直接读
                    curr = list(self.rc.get_ur5_angles())
                    curr[idx] = rad
                    self.rc.set_ur5_angles(tuple(curr))
                else:
                    self.rc.set_turntable_angle(rad)
            except Exception as e:
                logging.warning(f"指令发送失败: {e}")
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

    # --- 监控更新 (UI线程) ---
    def update_data(self, data):
        # 更新波形图
        joints = list(data['joints']) + [data['table']]
        self.plot_data = np.roll(self.plot_data, -1, axis=1)
        for i in range(7):
            self.plot_data[i, -1] = math.degrees(joints[i])
            self.curves[i].setData(self.plot_data[i])

        # 更新滑条 (仅当用户未操作时)
        if not any(s.isSliderDown() for s in self.sliders):
            self.programmatic_update = True
            for i in range(7):
                deg = math.degrees(joints[i])
                self.sliders[i].setValue(int(deg * 100))
                self.spinboxes[i].setValue(deg)
            self.programmatic_update = False

        # 轨迹线刷新
        if self.chk_show_traj.isChecked():
            # 注意：update_trail 内部也调用了 sim.getPose，需要锁吗？
            # 实际上 MonitorThread 已经在 lock 内完成了数据读取，
            # 但 RobotPath.update_trail 是在 backend 再次读取。
            # 最佳实践：MonitorThread 把 tip_pos 传出来，RobotPath 只负责画
            # 临时方案：加锁调用
            if self.zmq_mutex.tryLock(5):
                try:
                    self.path.update_trail()
                finally:
                    self.zmq_mutex.unlock()

    # --- IK 与 视点 (加锁保护) ---
    def on_ik_move_to_selected(self):
        if not self.rc: return

        items = self.vp_table.selectedItems()
        if not items: return

        handle = int(self.vp_table.item(items[0].row(), 0).text())

        self.zmq_mutex.lock()
        try:
            target_pos = self.rc.get_handle_position(handle)
            target_quat = self.rc.get_handle_quaternion(handle)
            solution = self.ik.solve(target_pos, target_quat, ref_frame=Frames.WORLD)

            if solution:
                self.rc.set_ur5_angles(solution)
                logging.info("IK 移动执行成功")
            else:
                QMessageBox.warning(self, "IK 失败", "目标不可达")
        except Exception as e:
            logging.error(f"IK Error: {e}")
        finally:
            self.zmq_mutex.unlock()

    # (其余函数如 on_create_manual_dummy, on_load_vp_file, on_export_matplotlib 保持逻辑不变，
    # 但凡涉及 self.rc 的调用，建议都加上 self.zmq_mutex.lock() / unlock())

    def on_create_manual_dummy(self):
        if not self.rc: return
        text = self.input_vp_pos.text()
        try:
            pos = [float(x) for x in text.split(',')]
            self.zmq_mutex.lock()
            try:
                h = self.rc.sim.createDummy(0.05)
                self.rc.sim.setObjectPosition(h, -1, pos)
                self.rc.sim.setObjectAlias(h, "Manual_Pt")

                row = self.vp_table.rowCount()
                self.vp_table.insertRow(row)
                self.vp_table.setItem(row, 0, QTableWidgetItem(str(h)))
                self.vp_table.setItem(row, 1, QTableWidgetItem("Manual_Pt"))
                self.vp_table.setItem(row, 2, QTableWidgetItem(str(pos)))
            finally:
                self.zmq_mutex.unlock()
        except Exception as e:
            logging.error(str(e))

    def on_toggle_traj(self, checked):
        if not self.rc: return
        self.zmq_mutex.lock()
        try:
            if checked:
                self.path.init_trail()
            else:
                self.path.clear_trail()
        finally:
            self.zmq_mutex.unlock()

    def on_clear_traj(self):
        self.monitor_thread.clear_trajectory()
        if self.rc:
            self.zmq_mutex.lock()
            try:
                self.path.clear_trail()
                self.path.init_trail()
            finally:
                self.zmq_mutex.unlock()

    # --- 业务逻辑续 (Matplotlib & 文件加载) ---
    def on_export_matplotlib(self):
        """导出学术风格3D轨迹图"""
        points = self.monitor_thread.trajectory_points
        if not points:
            QMessageBox.warning(self, "无数据", "没有记录到轨迹点，请先连接并运行机器人。")
            return

        try:
            data = np.array(points)  # N x 3

            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')

            # 绘制轨迹
            ax.plot(data[:, 0], data[:, 1], data[:, 2], label='End-Effector Trajectory', linewidth=1.5, color='#0072BD')

            # 绘制起点和终点
            ax.scatter(data[0, 0], data[0, 1], data[0, 2], c='green', marker='o', label='Start')
            ax.scatter(data[-1, 0], data[-1, 1], data[-1, 2], c='red', marker='x', label='End')

            # 学术风格设置
            ax.set_xlabel('X [m]', fontsize=12)
            ax.set_ylabel('Y [m]', fontsize=12)
            ax.set_zlabel('Z [m]', fontsize=12)
            ax.set_title('Robot End-Effector Trajectory', fontsize=14, fontweight='bold')
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend()

            # 自动调整视角
            ax.view_init(elev=25., azim=-45)
            plt.show()
            logging.info(f"轨迹图已生成，包含 {len(data)} 个采样点。")

        except Exception as e:
            logging.error(f"绘图失败: {e}")

    def on_load_vp_file(self):
        """从TXT文件加载视点"""
        if not self.rc:
            QMessageBox.warning(self, "未连接", "请先连接仿真软件。")
            return

        path, _ = QFileDialog.getOpenFileName(self, "选择视点文件", "", "Text Files (*.txt)")
        if not path: return

        self.zmq_mutex.lock()
        try:
            # 使用后端加载
            self.path.load_viewpoints_from_txt(path)
            self.path.create_visuals()

            # 刷新表格
            self.vp_table.setRowCount(0)
            for vp in self.path.viewpoints:
                row = self.vp_table.rowCount()
                self.vp_table.insertRow(row)
                # Column 0: Handle ID
                self.vp_table.setItem(row, 0, QTableWidgetItem(str(vp.handle)))
                # Column 1: Name
                self.vp_table.setItem(row, 1, QTableWidgetItem(vp.name))

                # Column 2: Position (获取实际生成的坐标)
                # 注意：RobotPath 中已经设置了 pose，这里直接读 backend 缓存或再查一次 sim 都可以
                # 为稳妥起见，读取 backend 的 local_matrix 转换后的值会更快，但这里演示查 sim
                pos = self.rc.get_handle_position(vp.handle)
                pos_str = f"[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
                self.vp_table.setItem(row, 2, QTableWidgetItem(pos_str))

            logging.info(f"成功加载文件: {path}, 共 {len(self.path.viewpoints)} 个视点")

        except Exception as e:
            logging.error(f"加载文件失败: {e}")
        finally:
            self.zmq_mutex.unlock()

    def close_app_cleanup(self):
        """退出前的清理"""
        logging.info("正在停止服务...")
        if self.monitor_thread.isRunning():
            self.monitor_thread.stop()

        # 最后的 ZMQ 操作
        self.zmq_mutex.lock()
        if self.rc:
            try:
                self.rc.stop()
                self.rc.close()
            except:
                pass
        self.zmq_mutex.unlock()
