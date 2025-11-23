# scan_qt/views/robot_main_window_slots.py
"""
机器人控制界面槽函数
"""

import math
import numpy as np
from typing import List
from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QMessageBox, QFileDialog, QTableWidgetItem

from scan_qt.robot.robot_comm import RobotComm, RobotCommError
from scan_qt.robot.robot_model import RobotModel
from scan_qt.robot.robot_ik import RobotIK, RobotIKError
from scan_qt.robot.robot_worker import RobotWorker
from scan_qt.robot.viewpoints_parser import ViewpointParser, ViewpointParseError


class RobotMainWindowSlots(QObject):
    """
    机器人控制界面槽函数类
    """

    def __init__(self, window):
        super().__init__(window)
        self.win = window  # RobotMainWindow 实例

    # ==================== 连接控制 ====================

    def on_connect_clicked(self):
        """连接按钮点击"""
        host = self.win.ui_components['edit_host'].text().strip()
        port_text = self.win.ui_components['edit_port'].text().strip()

        try:
            port = int(port_text)
        except ValueError:
            QMessageBox.warning(self.win, "错误", f"端口号无效: {port_text}")
            return

        try:
            # 创建通信对象
            self.win.comm = RobotComm(host, port)
            self.win.comm.connect()

            # 创建模型
            self.win.model = RobotModel()

            # 创建 IK 求解器
            self.win.ik_solver = RobotIK(self.win.comm, self.win.model)

            # 创建工作线程
            self.win.worker = RobotWorker(self.win.comm, self.win.model)
            self._connect_worker_signals()
            self.win.worker.start()

            # 更新 UI
            self.win.ui_components['btn_connect'].setEnabled(False)
            self.win.ui_components['btn_disconnect'].setEnabled(True)
            self.win.ui_components['label_connection_status'].setText("已连接")

            self.win.log_message(f"成功连接到 {host}:{port}", "SUCCESS")
            self.win.set_status("已连接")

            # 刷新初始状态
            self.on_refresh_frames_clicked()
            self.on_refresh_joints_clicked()

        except (RobotCommError, RobotIKError) as e:
            QMessageBox.critical(self.win, "连接失败", str(e))
            self.win.log_message(f"连接失败: {e}", "ERROR")
        except Exception as e:
            QMessageBox.critical(self.win, "异常", f"未知错误: {e}")
            self.win.log_message(f"连接异常: {e}", "ERROR")

    def on_disconnect_clicked(self):
        """断开按钮点击"""
        try:
            # 停止工作线程
            if self.win.worker is not None:
                self.win.worker.stop()
                self.win.worker = None

            # 断开连接
            if self.win.comm is not None:
                self.win.comm.disconnect()
                self.win.comm = None

            self.win.ik_solver = None
            self.win.model = None

            # 更新 UI
            self.win.ui_components['btn_connect'].setEnabled(True)
            self.win.ui_components['btn_disconnect'].setEnabled(False)
            self.win.ui_components['label_connection_status'].setText("未连接")

            self.win.log_message("已断开连接", "INFO")
            self.win.set_status("未连接")

        except Exception as e:
            QMessageBox.warning(self.win, "警告", f"断开连接时异常: {e}")
            self.win.log_message(f"断开异常: {e}", "WARNING")

    def _connect_worker_signals(self):
        """连接工作线程信号"""
        self.win.worker.progress_updated.connect(self.win.update_progress)
        self.win.worker.status_updated.connect(lambda msg: self.win.set_status(msg, True))
        self.win.worker.motion_finished.connect(self._on_motion_finished)
        self.win.worker.joint_state_updated.connect(self._on_joint_state_updated)

    def _on_motion_finished(self, success: bool):
        """运动完成回调"""
        if success:
            self.win.log_message("运动执行完成", "SUCCESS")
            self.win.set_status("就绪", False)
        else:
            self.win.log_message("运动执行失败", "ERROR")
            self.win.set_status("运动失败", False)

    def _on_joint_state_updated(self, ur5_config: List[float], turtle_config: float):
        """关节状态更新回调"""
        # 更新滑条和数值显示
        joint_names = [f"joint{i}" for i in range(1, 7)] + ["turtle_joint"]
        configs = ur5_config + [turtle_config]

        for name, config_rad in zip(joint_names, configs):
            config_deg = math.degrees(config_rad)

            # 更新滑条
            slider = self.win.ui_components['sliders_joints'].get(name)
            if slider is not None:
                slider.blockSignals(True)
                slider.setValue(int(round(config_deg)))
                slider.blockSignals(False)

            # 更新数值显示
            label = self.win.ui_components['labels_joint_values'].get(name)
            if label is not None:
                label.setText(f"{config_deg:.1f}°")

    # ==================== 视点管理 ====================

    def on_load_viewpoints_clicked(self):
        """加载视点文件"""
        filepath, _ = QFileDialog.getOpenFileName(
            self.win,
            "选择视点文件",
            "",
            "文本文件 (*.txt);;所有文件 (*.*)"
        )

        if not filepath:
            return

        try:
            # 解析视点文件
            self.win.viewpoints = ViewpointParser.parse_file(filepath)

            # 更新 UI
            self._update_viewpoint_table()
            self.win.ui_components['label_viewpoint_file'].setText(filepath.split('/')[-1])

            self.win.log_message(f"成功加载 {len(self.win.viewpoints)} 个视点", "SUCCESS")

            # 更新轨迹信息
            self.win.ui_components['label_trajectory_count'].setText(str(len(self.win.viewpoints)))

        except ViewpointParseError as e:
            QMessageBox.critical(self.win, "解析失败", str(e))
            self.win.log_message(f"视点解析失败: {e}", "ERROR")
        except Exception as e:
            QMessageBox.critical(self.win, "异常", f"加载视点时异常: {e}")
            self.win.log_message(f"加载异常: {e}", "ERROR")

    def on_add_manual_viewpoint_clicked(self):
        """添加手动输入的视点"""
        text = self.win.ui_components['edit_manual_viewpoint'].text().strip()

        if not text:
            QMessageBox.information(self.win, "提示", "请输入视点数据")
            return

        try:
            # 解析单个视点
            name, position, direction = ViewpointParser.parse_single_viewpoint(text)

            # 添加到列表
            self.win.viewpoints.append((name, position, direction))

            # 更新 UI
            self._update_viewpoint_table()
            self.win.ui_components['edit_manual_viewpoint'].clear()

            self.win.log_message(f"添加视点: {name}", "SUCCESS")

            # 更新轨迹信息
            self.win.ui_components['label_trajectory_count'].setText(str(len(self.win.viewpoints)))

        except ViewpointParseError as e:
            QMessageBox.warning(self.win, "解析失败", str(e))
            self.win.log_message(f"视点解析失败: {e}", "WARNING")

    def _update_viewpoint_table(self):
        """更新视点表格"""
        table = self.win.ui_components['table_viewpoints']
        table.setRowCount(len(self.win.viewpoints))

        for i, (name, position, direction) in enumerate(self.win.viewpoints):
            # 名称
            table.setItem(i, 0, QTableWidgetItem(name))

            # 位置
            pos_str = f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
            table.setItem(i, 1, QTableWidgetItem(pos_str))

            # 方向
            dir_str = f"({direction[0]:.2f}, {direction[1]:.2f}, {direction[2]:.2f})"
            table.setItem(i, 2, QTableWidgetItem(dir_str))

            # 状态
            table.setItem(i, 3, QTableWidgetItem("待求解"))

    def on_create_dummies_clicked(self):
        """创建 Dummy 对象"""
        if not self._ensure_connected():
            return

        if not self.win.viewpoints:
            QMessageBox.information(self.win, "提示", "没有可用的视点")
            return

        try:
            # 创建 Dummy
            self.win.comm.create_dummies_from_viewpoints(self.win.viewpoints)

            self.win.log_message(f"已创建 {len(self.win.viewpoints)} 个 Dummy", "SUCCESS")

        except RobotCommError as e:
            QMessageBox.critical(self.win, "错误", str(e))
            self.win.log_message(f"创建 Dummy 失败: {e}", "ERROR")

    def on_clear_dummies_clicked(self):
        """清除 Dummy 对象"""
        if not self._ensure_connected():
            return

        try:
            self.win.comm.clear_all_dummies()
            self.win.log_message("已清除所有 Dummy", "INFO")

        except RobotCommError as e:
            QMessageBox.warning(self.win, "警告", str(e))
            self.win.log_message(f"清除 Dummy 失败: {e}", "WARNING")

    # ==================== 坐标系 ====================

    def on_refresh_frames_clicked(self):
        """刷新坐标系信息"""
        if not self._ensure_connected():
            return

        try:
            frames = {
                "W - 世界": self.win.comm.get_T_W(),
                "B - 基座": self.win.comm.get_T_WB(),
                "J - 转台": self.win.comm.get_T_WJ(),
                "O - 工件": self.win.comm.get_T_WO(),
                "S - 扫描仪": self.win.comm.get_T_WS(),
            }

            table = self.win.ui_components['table_frames']

            for i, (name, T) in enumerate(frames.items()):
                pos, euler = self.win.comm.matrix_to_pos_euler(T)
                euler_deg = np.degrees(euler)

                pose_str = (
                    f"pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) | "
                    f"rpy: ({euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°)"
                )

                table.setItem(i, 1, QTableWidgetItem(pose_str))

            self.win.log_message("坐标系信息已更新", "INFO")

        except RobotCommError as e:
            QMessageBox.critical(self.win, "错误", str(e))
            self.win.log_message(f"刷新坐标系失败: {e}", "ERROR")

    # ==================== 关节控制 ====================

    def on_refresh_joints_clicked(self):
        """刷新关节状态"""
        if not self._ensure_connected():
            return

        try:
            ur5_config, turtle_config = self.win.comm.get_all_joint_positions()
            self._on_joint_state_updated(ur5_config, turtle_config)

            self.win.log_message("关节状态已更新", "INFO")

        except RobotCommError as e:
            QMessageBox.critical(self.win, "错误", str(e))
            self.win.log_message(f"刷新关节失败: {e}", "ERROR")

    def on_go_home_clicked(self):
        """回到 Home 位姿"""
        if not self._ensure_connected():
            return

        try:
            duration = self.win.ui_components['spin_motion_duration'].value()

            self.win.worker.move_to_config(
                self.win.model.home_ur5_rad,
                self.win.model.home_turtle_rad,
                duration
            )

            self.win.log_message("正在返回 Home 位姿", "INFO")

        except Exception as e:
            QMessageBox.critical(self.win, "错误", f"运动失败: {e}")
            self.win.log_message(f"Home 运动失败: {e}", "ERROR")

    def on_joint_slider_changed(self, joint_name: str, value_deg: int):
        """关节滑条变化"""
        if not self._ensure_connected():
            return

        try:
            # 更新数值显示
            label = self.win.ui_components['labels_joint_values'].get(joint_name)
            if label is not None:
                label.setText(f"{value_deg:.1f}°")

            # 转换为弧度并限幅
            value_rad = math.radians(float(value_deg))
            value_rad = self.win.model.clamp_joint_rad(joint_name, value_rad)

            # 下发命令
            self.win.comm.set_joint_position(joint_name, value_rad)

        except Exception as e:
            self.win.log_message(f"关节控制异常: {e}", "WARNING")

    # ==================== IK 求解 ====================

    def on_solve_ik_selected_clicked(self):
        """求解选中视点的 IK"""
        if not self._ensure_connected():
            return

        # 获取选中行
        table = self.win.ui_components['table_viewpoints']
        selected_rows = table.selectionModel().selectedRows()

        if not selected_rows:
            QMessageBox.information(self.win, "提示", "请先选择视点")
            return

        try:
            approach_distance = self.win.ui_components['spin_approach_distance'].value() / 1000.0

            for row in selected_rows:
                idx = row.row()
                name, position, direction = self.win.viewpoints[idx]

                # 求解 IK
                success, ur5_config, turtle_config = self.win.ik_solver.solve_ik_for_viewpoint(
                    position, direction, approach_distance
                )

                # 更新结果
                self._update_ik_result(idx, name, success, ur5_config, turtle_config)

                # 更新视点表格状态
                status = "求解成功" if success else "求解失败"
                table.setItem(idx, 3, QTableWidgetItem(status))

            self.win.log_message(f"已求解 {len(selected_rows)} 个视点", "SUCCESS")

        except RobotIKError as e:
            QMessageBox.critical(self.win, "IK 求解失败", str(e))
            self.win.log_message(f"IK 求解失败: {e}", "ERROR")

    def on_solve_ik_all_clicked(self):
        """批量求解所有视点的 IK"""
        if not self._ensure_connected():
            return

        if not self.win.viewpoints:
            QMessageBox.information(self.win, "提示", "没有可用的视点")
            return

        try:
            approach_distance = self.win.ui_components['spin_approach_distance'].value() / 1000.0

            # 批量求解
            self.win.ik_results = self.win.ik_solver.solve_ik_batch(
                self.win.viewpoints,
                approach_distance
            )

            # 更新 UI
            self._update_all_ik_results()

            # 统计成功数量
            success_count = sum(1 for _, s, _, _ in self.win.ik_results if s)

            self.win.log_message(
                f"批量求解完成: {success_count}/{len(self.win.viewpoints)} 成功",
                "SUCCESS"
            )

            # 更新轨迹信息
            self.win.ui_components['label_trajectory_success'].setText(str(success_count))

        except RobotIKError as e:
            QMessageBox.critical(self.win, "IK 求解失败", str(e))
            self.win.log_message(f"批量求解失败: {e}", "ERROR")

    def _update_ik_result(self, idx: int, name: str, success: bool,
                          ur5_config: List[float], turtle_config: float):
        """更新单个 IK 结果"""
        # 更新或添加到结果列表
        result = (name, success, ur5_config, turtle_config)

        # 查找是否已存在
        found = False
        for i, (n, _, _, _) in enumerate(self.win.ik_results):
            if n == name:
                self.win.ik_results[i] = result
                found = True
                break

        if not found:
            self.win.ik_results.append(result)

        # 更新表格
        table = self.win.ui_components['table_ik_results']

        # 查找表格中的行
        row = -1
        for i in range(table.rowCount()):
            if table.item(i, 0) and table.item(i, 0).text() == name:
                row = i
                break

        if row == -1:
            row = table.rowCount()
            table.insertRow(row)

        # 更新单元格
        table.setItem(row, 0, QTableWidgetItem(name))

        status = "✓ 成功" if success else "✗ 失败"
        table.setItem(row, 1, QTableWidgetItem(status))

        if success:
            config_str = f"UR5: [{', '.join([f'{math.degrees(q):.1f}' for q in ur5_config])}]° | Turtle: {math.degrees(turtle_config):.1f}°"
            table.setItem(row, 2, QTableWidgetItem(config_str))
        else:
            table.setItem(row, 2, QTableWidgetItem("N/A"))

    def _update_all_ik_results(self):
        """更新所有 IK 结果"""
        table_vp = self.win.ui_components['table_viewpoints']
        table_ik = self.win.ui_components['table_ik_results']

        # 清空 IK 结果表格
        table_ik.setRowCount(0)

        for idx, (name, success, ur5_config, turtle_config) in enumerate(self.win.ik_results):
            # 更新视点表格状态
            status = "求解成功" if success else "求解失败"
            table_vp.setItem(idx, 3, QTableWidgetItem(status))

            # 添加到 IK 结果表格
            row = table_ik.rowCount()
            table_ik.insertRow(row)

            table_ik.setItem(row, 0, QTableWidgetItem(name))

            status_text = "✓ 成功" if success else "✗ 失败"
            table_ik.setItem(row, 1, QTableWidgetItem(status_text))

            if success:
                config_str = f"UR5: [{', '.join([f'{math.degrees(q):.1f}' for q in ur5_config])}]° | Turtle: {math.degrees(turtle_config):.1f}°"
                table_ik.setItem(row, 2, QTableWidgetItem(config_str))
            else:
                table_ik.setItem(row, 2, QTableWidgetItem("N/A"))

    # ==================== 运动执行 ====================

    def on_move_to_selected_clicked(self):
        """运动到选中的视点"""
        if not self._ensure_connected():
            return

        # 获取选中行
        table = self.win.ui_components['table_ik_results']
        selected_rows = table.selectionModel().selectedRows()

        if not selected_rows:
            QMessageBox.information(self.win, "提示", "请先选择 IK 结果")
            return

        if len(selected_rows) > 1:
            QMessageBox.information(self.win, "提示", "请只选择一个视点")
            return

        try:
            idx = selected_rows[0].row()

            if idx >= len(self.win.ik_results):
                QMessageBox.warning(self.win, "警告", "无效的选择")
                return

            name, success, ur5_config, turtle_config = self.win.ik_results[idx]

            if not success:
                QMessageBox.warning(self.win, "警告", f"视点 {name} 的 IK 求解失败")
                return

            # 执行运动
            duration = self.win.ui_components['spin_motion_duration'].value()

            self.win.worker.move_to_config(ur5_config, turtle_config, duration)

            self.win.log_message(f"正在运动到视点: {name}", "INFO")

        except Exception as e:
            QMessageBox.critical(self.win, "错误", f"运动失败: {e}")
            self.win.log_message(f"运动失败: {e}", "ERROR")

    def on_execute_trajectory_clicked(self):
        """执行完整轨迹"""
        if not self._ensure_connected():
            return

        if not self.win.ik_results:
            QMessageBox.information(self.win, "提示", "请先求解 IK")
            return

        # 过滤出成功的结果
        successful_configs = [
            ur5_config + [turtle_config]
            for _, success, ur5_config, turtle_config in self.win.ik_results
            if success
        ]

        if not successful_configs:
            QMessageBox.warning(self.win, "警告", "没有成功求解的视点")
            return

        try:
            duration_per_segment = self.win.ui_components['spin_motion_duration'].value()

            # 执行轨迹
            self.win.worker.execute_trajectory(successful_configs, duration_per_segment)

            self.win.log_message(
                f"开始执行轨迹 ({len(successful_configs)} 个点)",
                "INFO"
            )

        except Exception as e:
            QMessageBox.critical(self.win, "错误", f"轨迹执行失败: {e}")
            self.win.log_message(f"轨迹执行失败: {e}", "ERROR")

    # ==================== 轨迹可视化 ====================

    def on_trajectory_visibility_changed(self, state: int):
        """轨迹可见性变化"""
        if state == 2:  # Checked
            self.on_update_trajectory_clicked()
        else:
            self.on_clear_trajectory_clicked()

    def on_update_trajectory_clicked(self):
        """更新轨迹显示"""
        if not self._ensure_connected():
            return

        if not self.win.ik_results:
            QMessageBox.information(self.win, "提示", "请先求解 IK")
            return

        try:
            # 提取成功的位置点
            positions = []
            for name, success, _, _ in self.win.ik_results:
                if success:
                    # 从视点列表中找到对应的位置
                    for vp_name, position, _ in self.win.viewpoints:
                        if vp_name == name:
                            positions.append(position)
                            break

            if not positions:
                QMessageBox.information(self.win, "提示", "没有成功求解的视点")
                return

            # 创建轨迹线
            self.win.comm.create_trajectory_line(positions, [0.0, 1.0, 0.0])

            # 计算路径长度
            total_length = 0.0
            for i in range(len(positions) - 1):
                total_length += np.linalg.norm(
                    np.array(positions[i+1]) - np.array(positions[i])
                )

            self.win.ui_components['label_trajectory_length'].setText(f"{total_length:.3f} m")

            self.win.log_message("轨迹线已更新", "SUCCESS")

        except RobotCommError as e:
            QMessageBox.critical(self.win, "错误", str(e))
            self.win.log_message(f"更新轨迹失败: {e}", "ERROR")

    def on_clear_trajectory_clicked(self):
        """清除轨迹显示"""
        if not self._ensure_connected():
            return

        try:
            self.win.comm.clear_trajectory_line()
            self.win.log_message("轨迹线已清除", "INFO")

        except RobotCommError as e:
            QMessageBox.warning(self.win, "警告", str(e))
            self.win.log_message(f"清除轨迹失败: {e}", "WARNING")

    # ==================== 日志 ====================

    def on_clear_log_clicked(self):
        """清除日志"""
        self.win.ui_components['text_log'].clear()

    # ==================== 工具方法 ====================

    def _ensure_connected(self) -> bool:
        """确保已连接"""
        if self.win.comm is None or not self.win.comm.is_connected():
            QMessageBox.information(self.win, "提示", "请先连接 CoppeliaSim")
            return False
        return True
