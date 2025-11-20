# scan_qt/views/robot_main_window_slots.py
import math
import numpy as np

from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QMessageBox
from scan_qt.coppeliasim import sim


from scan_qt.robot.robot_comm import RobotComm, RobotCommError


class RobotMainWindowSlots(QObject):
    """
    专门存放 RobotMainWidget 的槽函数。
    通过 self.win 访问 UI 控件、RobotComm、RobotPlanner。
    """

    def __init__(self, win):
        super().__init__(win)
        self.win = win  # RobotMainWidget 实例

    # ------------ 连接 / 断开 ------------

    def on_connect_clicked(self):
        host = self.win.edit_host.text().strip()
        port_text = self.win.edit_port.text().strip()
        try:
            port = int(port_text)
        except ValueError:
            QMessageBox.warning(
                self.win, "错误", f"端口号无效: {port_text}"
            )
            return

        try:
            comm = RobotComm(host=host, port=port, connect_immediately=True)
        except RobotCommError as e:
            QMessageBox.critical(
                self.win, "连接失败", str(e)
            )
            self.win.set_status_text("连接失败")
            return
        except Exception as e:
            QMessageBox.critical(
                self.win, "连接异常", repr(e)
            )
            self.win.set_status_text("连接异常")
            return

        # 通讯对象保存到界面，并初始化 RobotModel & RobotPlanner
        self.win.on_connected(comm)

        # 初次连接后自动刷新一次坐标系/关节信息
        try:
            self.win.update_frames_display()
            self.win.update_joint_display()
        except RobotCommError as e:
            QMessageBox.critical(self.win, "读取错误", str(e))
        except Exception as e:
            QMessageBox.critical(self.win, "读取异常", repr(e))

    def on_disconnect_clicked(self):
        self.win.on_disconnected()

    # ------------ 坐标系相关 ------------

    def on_refresh_frames_clicked(self):
        if self.win.comm is None or not self.win.comm.is_connected():
            QMessageBox.information(
                self.win, "提示", "尚未连接 CoppeliaSim。"
            )
            return

        try:
            self.win.update_frames_display()
            # 同时在控制台打印
            self.win.comm.print_frames_info()
        except RobotCommError as e:
            QMessageBox.critical(self.win, "错误", str(e))
        except Exception as e:
            QMessageBox.critical(self.win, "异常", repr(e))

    # ------------ 坐标变换测试：点 ------------

    def on_transform_point_clicked(self):
        if self.win.comm is None or not self.win.comm.is_connected():
            QMessageBox.information(
                self.win, "提示", "尚未连接 CoppeliaSim。"
            )
            return

        from_frame = self.win.combo_from_frame.currentText()
        to_frame = self.win.combo_to_frame.currentText()

        p = np.array([
            self.win.spin_px.value(),
            self.win.spin_py.value(),
            self.win.spin_pz.value(),
        ], dtype=float)

        try:
            q = self.win.comm.transform_point(p, from_frame, to_frame)
        except RobotCommError as e:
            QMessageBox.critical(self.win, "错误", str(e))
            return
        except Exception as e:
            QMessageBox.critical(self.win, "异常", repr(e))
            return

        print(
            f"[RobotQt] Point {from_frame}->{to_frame}: "
            f"p_{from_frame}={p}, p_{to_frame}={q}"
        )
        QMessageBox.information(
            self.win,
            "转换结果",
            f"p_{from_frame} = ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})\n"
            f"p_{to_frame} = ({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f})"
        )

    # ------------ 坐标变换测试：方向 ------------

    def on_transform_dir_clicked(self):
        if self.win.comm is None or not self.win.comm.is_connected():
            QMessageBox.information(
                self.win, "提示", "尚未连接 CoppeliaSim。"
            )
            return

        from_frame = self.win.combo_from_frame.currentText()
        to_frame = self.win.combo_to_frame.currentText()

        v = np.array([
            self.win.spin_vx.value(),
            self.win.spin_vy.value(),
            self.win.spin_vz.value(),
        ], dtype=float)

        try:
            w = self.win.comm.transform_direction(v, from_frame, to_frame)
        except RobotCommError as e:
            QMessageBox.critical(self.win, "错误", str(e))
            return
        except Exception as e:
            QMessageBox.critical(self.win, "异常", repr(e))
            return

        print(
            f"[RobotQt] Direction {from_frame}->{to_frame}: "
            f"v_{from_frame}={v}, v_{to_frame}={w}"
        )
        QMessageBox.information(
            self.win,
            "转换结果",
            f"v_{from_frame} = ({v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f})\n"
            f"v_{to_frame} = ({w[0]:.3f}, {w[1]:.3f}, {w[2]:.3f})"
        )

    # ------------ 关节状态与规划 ------------

    def _ensure_planner(self) -> bool:
        if self.win.robot_planner is None or self.win.robot_model is None:
            QMessageBox.information(
                self.win, "提示", "尚未连接 CoppeliaSim 或规划器未初始化。"
            )
            return False
        return True

    def on_refresh_joints_clicked(self):
        if not self._ensure_planner():
            return
        try:
            self.win.update_joint_display()
        except RobotCommError as e:
            QMessageBox.critical(self.win, "读取关节错误", str(e))
        except Exception as e:
            QMessageBox.critical(self.win, "读取关节异常", repr(e))

    def on_go_home_clicked(self):
        if not self._ensure_planner():
            return
        try:
            self.win.robot_planner.go_home()
            self.win.update_joint_display()
        except RobotCommError as e:
            QMessageBox.critical(self.win, "运动错误", str(e))
        except Exception as e:
            QMessageBox.critical(self.win, "运动异常", repr(e))

    def on_move_to_target_clicked(self):
        if not self._ensure_planner():
            return

        model = self.win.robot_model
        planner = self.win.robot_planner

        # 从 UI 读目标关节角（deg -> rad）
        q_target_deg = []
        for name in model.ur5_joint_names:
            sp = self.win.spin_target_joints.get(name)
            q_target_deg.append(sp.value() if sp is not None else 0.0)
        turtle_target_deg = self.win.spin_target_joints.get(
            model.turtle_joint_name
        ).value()

        q_target_rad = [math.radians(v) for v in q_target_deg]
        turtle_target_rad = math.radians(turtle_target_deg)

        try:
            planner.move_to_config(q_target_rad, turtle_target_rad)
            self.win.update_joint_display()
        except RobotCommError as e:
            QMessageBox.critical(self.win, "运动错误", str(e))
        except Exception as e:
            QMessageBox.critical(self.win, "运动异常", repr(e))

    def on_go_yaw_clicked(self):
        if not self._ensure_planner():
            return

        planner = self.win.robot_planner
        yaw_deg = self.win.spin_total_yaw_deg.value()
        yaw_rad = math.radians(yaw_deg)

        try:
            planner.go_to_yaw(yaw_rad)
            self.win.update_joint_display()
        except RobotCommError as e:
            QMessageBox.critical(self.win, "yaw 分配运动错误", str(e))
        except Exception as e:
            QMessageBox.critical(self.win, "yaw 分配运动异常", repr(e))

    # ------------ 关节 JOG 滑条控制 ------------

    def on_joint_slider_changed(self, joint_name: str, value_deg: int):
        """
        滑条拖动时实时下发该关节的目标角度（deg）：
        - 转成 rad
        - 做一次限幅
        - 直接调用 simxSetJointTargetPosition（position control 模式）
        """

        if not self._ensure_planner():
            return

        comm = self.win.comm
        model = self.win.robot_model
        planner = self.win.robot_planner

        # 更新 jog 值 label
        lbl = self.win.labels_jog_value.get(joint_name)
        if lbl is not None:
            lbl.setText(f"{value_deg:.1f}°")

        # 转成 rad 并做限幅
        value_rad = math.radians(float(value_deg))
        value_rad = model.clamp_joint_rad(joint_name, value_rad)

        try:
            if joint_name in model.ur5_joint_names:
                # UR5 关节：根据名称找到对应 handle
                idx = model.ur5_joint_names.index(joint_name)
                jh = comm.ur5_joints[idx]
                sim.simxSetJointTargetPosition(
                    comm.client_id, jh, value_rad, sim.simx_opmode_oneshot
                )
            elif joint_name == model.turtle_joint_name:
                # 转台关节
                sim.simxSetJointTargetPosition(
                    comm.client_id, comm.handle_turtle, value_rad, sim.simx_opmode_oneshot
                )
            else:
                return
        except Exception as e:
            # 不弹框，避免拖动滑条时频繁弹窗，只在控制台提示
            print(f"[RobotQt] JOG 关节 {joint_name} 下发异常:", e)
            return

        # 同步“当前角度”显示和“目标关节角 spinbox”
        try:
            # 先刷新一次当前关节（从仿真读）
            self.win.update_joint_display()
        except Exception as e:
            print("[RobotQt] 更新关节显示异常:", e)

        # 再把目标关节 spinbox 更新到当前 slider 值
        sp = self.win.spin_target_joints.get(joint_name)
        if sp is not None:
            sp.blockSignals(True)
            sp.setValue(float(value_deg))
            sp.blockSignals(False)
