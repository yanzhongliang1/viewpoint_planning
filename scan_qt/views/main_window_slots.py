# scan_qt/views/main_window_slots.py
import os
import numpy as np

from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import (
    QFileDialog,
    QTableWidgetItem,
    QCheckBox,
)


class MainWindowSlots(QObject):
    """
    专门存放 MainWindow 的槽函数，通过 self.win 访问 MainWindow 中的控件和 controller。
    """

    def __init__(self, win):
        super().__init__(win)
        self.win = win  # MainWindow 实例

    # =========================
    # 文件操作
    # =========================
    def open_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self.win,
            "选择 PLY 模型文件",
            "",
            "PLY 文件 (*.ply);;所有文件 (*.*)",
        )
        if filename:
            ok = self.win.controller.open_ply(filename)
            self.win.file_label.setText(os.path.basename(filename) if ok else "加载失败")

    def save_file(self):
        filename, _ = QFileDialog.getSaveFileName(
            self.win,
            "保存当前几何为 PLY",
            "",
            "PLY 文件 (*.ply)"
        )
        if filename:
            if not filename.lower().endswith(".ply"):
                filename += ".ply"
            ok = self.win.controller.save_ply(filename)
            self.win.file_label.setText(
                "已保存: " + os.path.basename(filename) if ok else "保存失败"
            )

    # =========================
    # 显示与包围盒
    # =========================
    def on_bbox_toggled(self, state):
        self.win.controller.toggle_bbox(bool(state))

    def on_point_size_changed(self, value):
        self.win.controller.set_point_size(float(value))

    # =========================
    # 点云处理
    # =========================
    def on_use_pcd_toggled(self, state):
        num_points = self.win.spin_sample_points.value()
        self.win.controller.toggle_use_sampled_pcd(bool(state), num_points=num_points)

    def on_apply_downsample(self):
        voxel_size = self.win.spin_down_voxel.value()
        self.win.controller.apply_voxel_downsample(voxel_size)

    # =========================
    # 体素分割显示
    # =========================
    def on_voxel_toggled(self, state):
        self.win.controller.toggle_voxel_grid(bool(state))

    def on_voxel_size_changed(self, value):
        self.win.controller.set_voxel_size(float(value))

    # =========================
    # 法向显示
    # =========================
    def _get_normal_color_from_combo(self):
        text = self.win.combo_normal_color.currentText()
        if text == "红":
            return (1.0, 0.0, 0.0)
        elif text == "绿":
            return (0.0, 1.0, 0.0)
        elif text == "蓝":
            return (0.0, 0.0, 1.0)
        elif text == "白":
            return (1.0, 1.0, 1.0)
        else:
            return (1.0, 0.0, 0.0)

    def on_show_normals_toggled(self, state):
        self.win.controller.toggle_show_normals(bool(state))

    def on_normal_params_changed(self, *args):
        length = self.win.spin_normal_length.value()
        step = self.win.spin_normal_step.value()
        color = self._get_normal_color_from_combo()
        self.win.controller.set_normal_params(length=length, step=step, color=color)

    # =========================
    # 视点切换
    # =========================
    def change_view(self, view_name: str):
        self.win.controller.set_view(view_name)

    # =========================
    # Camera / 扫描相关
    # =========================
    def _get_scan_color_from_combo(self):
        text = self.win.combo_scan_color.currentText()
        if text == "红":
            return (1.0, 0.0, 0.0)
        elif text == "绿":
            return (0.0, 1.0, 0.0)
        elif text == "蓝":
            return (0.0, 0.0, 1.0)
        elif text == "黄":
            return (1.0, 1.0, 0.0)
        elif text == "白":
            return (1.0, 1.0, 1.0)
        elif text == "自动":
            return None  # 交给 controller 的颜色循环策略
        else:
            return None

    def on_update_frustum_clicked(self):
        # 从 UI 读参数写入 camera_model
        fov = self.win.spin_cam_fov.value()
        near = self.win.spin_cam_near.value()
        far = self.win.spin_cam_far.value()
        best = self.win.spin_cam_best.value()

        pos = (
            self.win.spin_cam_posx.value(),
            self.win.spin_cam_posy.value(),
            self.win.spin_cam_posz.value(),
        )
        direction = (
            self.win.spin_cam_dirx.value(),
            self.win.spin_cam_diry.value(),
            self.win.spin_cam_dirz.value(),
        )

        self.win.camera_controller.set_intrinsics(
            fov_deg=fov, near=near, far=far, best_distance=best
        )
        self.win.camera_controller.set_pose(
            pos=np.array(pos),
            direction=np.array(direction),
        )

        self.win.camera_controller.update_frustum()

    def on_scan_once_clicked(self):
        color = self._get_scan_color_from_combo()
        use_occ = self.win.cb_use_occlusion.isChecked()
        self.win.camera_controller.scan_once(color=color, use_occlusion=use_occ)
        self.refresh_view_table()

    # =========================
    # 视点 / 扫描帧表格
    # =========================
    def refresh_view_table(self):
        win = self.win
        views = win.camera_controller.list_views()
        win.view_table.blockSignals(True)  # 避免刷新时触发信号
        win.view_table.setRowCount(len(views))

        for row, info in enumerate(views):
            # 名称
            item_name = QTableWidgetItem(info["name"])
            win.view_table.setItem(row, 0, item_name)

            # 可见性复选框
            cb = QCheckBox()
            cb.setChecked(info["visible"])
            cb.stateChanged.connect(
                lambda state, r=row: self.on_view_visibility_changed(r, state)
            )
            win.view_table.setCellWidget(row, 1, cb)

            # 颜色文本
            color = info["color"]
            color_text = f"({color[0]:.2f},{color[1]:.2f},{color[2]:.2f})"
            item_color = QTableWidgetItem(color_text)
            win.view_table.setItem(row, 2, item_color)

        win.view_table.blockSignals(False)

    def on_view_visibility_changed(self, row, state):
        visible = bool(state)
        self.win.camera_controller.set_view_visibility(row, visible)

    def on_delete_selected_view(self):
        win = self.win
        row = win.view_table.currentRow()
        if row < 0:
            return
        win.camera_controller.remove_view(row)
        self.refresh_view_table()

    def on_grab_camera_clicked(self):
        """
        从当前 Open3D 视图读取相机位置/方向，并更新右侧参数框。
        """
        win = self.win
        win.camera_controller.grab_pose_from_current_view()

        cam = win.camera_model
        win.spin_cam_posx.setValue(cam.position[0])
        win.spin_cam_posy.setValue(cam.position[1])
        win.spin_cam_posz.setValue(cam.position[2])

        win.spin_cam_dirx.setValue(cam.direction[0])
        win.spin_cam_diry.setValue(cam.direction[1])
        win.spin_cam_dirz.setValue(cam.direction[2])

        win.spin_cam_fov.setValue(cam.fov_deg)
        win.spin_cam_near.setValue(cam.near)
        win.spin_cam_far.setValue(cam.far)
        win.spin_cam_best.setValue(cam.best_distance)
        print("UI 更新后的 camera pos:", cam.position)

    def on_view_table_double_clicked(self, item):
        """
        双击某个视点：相机跳转到该视点，并刷新右侧 Camera 参数。
        """
        win = self.win
        row = item.row()
        win.camera_controller.go_to_view(row)

        cam = win.camera_model
        win.spin_cam_posx.setValue(cam.position[0])
        win.spin_cam_posy.setValue(cam.position[1])
        win.spin_cam_posz.setValue(cam.position[2])
        win.spin_cam_dirx.setValue(cam.direction[0])
        win.spin_cam_diry.setValue(cam.direction[1])
        win.spin_cam_dirz.setValue(cam.direction[2])
        win.spin_cam_fov.setValue(cam.fov_deg)
        win.spin_cam_near.setValue(cam.near)
        win.spin_cam_far.setValue(cam.far)
        win.spin_cam_best.setValue(cam.best_distance)

    def on_save_views_clicked(self):
        win = self.win
        filename, _ = QFileDialog.getSaveFileName(
            win,
            "保存视点记录到 txt",
            "",
            "Text 文件 (*.txt)"
        )
        if not filename:
            return
        if not filename.lower().endswith(".txt"):
            filename += ".txt"
        win.camera_controller.save_views_to_txt(filename)

    def on_show_only_selected_scan(self):
        win = self.win
        row = win.view_table.currentRow()
        if row < 0:
            return

        views = win.camera_controller.list_views()
        for i in range(len(views)):
            win.camera_controller.set_view_visibility(i, i == row)
        self.refresh_view_table()

    def on_export_selected_scan(self):
        win = self.win
        row = win.view_table.currentRow()
        if row < 0:
            return
        filename, _ = QFileDialog.getSaveFileName(
            win,
            "导出选中扫描帧为 PLY",
            "",
            "PLY 文件 (*.ply)"
        )
        if not filename:
            return
        if not filename.lower().endswith(".ply"):
            filename += ".ply"
        win.camera_controller.export_scan_pcd(row, filename)

    def on_export_all_scans(self):
        win = self.win
        filename, _ = QFileDialog.getSaveFileName(
            win,
            "导出所有扫描点云为 PLY",
            "",
            "PLY 文件 (*.ply)"
        )
        if not filename:
            return
        if not filename.lower().endswith(".ply"):
            filename += ".ply"
        win.camera_controller.export_all_scans(filename)

    # ----------- 关闭事件 -----------

    def closeEvent(self, event):
        self.win.controller.close()
        event.accept()