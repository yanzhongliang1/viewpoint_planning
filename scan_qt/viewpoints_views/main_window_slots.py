import os
import numpy as np

from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import (
    QFileDialog,
    QTableWidgetItem,
    QCheckBox,
    QWidget,
    QHBoxLayout  # 确保 Table Cell Widget 需要的 import
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
            self.win.logger.log(f"正在加载模型: {filename} ...", "INFO")
            ok = self.win.controller.open_ply(filename)
            if ok:
                name = os.path.basename(filename)
                self.win.file_label.setText(name)
                self.win.logger.log(f"模型加载成功: {name}", "SUCCESS")

                # 尝试嵌入 Open3D 窗口 (如果在 Windows 下)
                # 注意：这需要 ModelView.ensure_window() 已经被调用
                # self.embed_window_if_possible() 
            else:
                self.win.file_label.setText("加载失败")
                self.win.logger.log("模型加载失败，请检查文件格式。", "ERROR")

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
            if ok:
                self.win.logger.log(f"文件已保存: {filename}", "SUCCESS")
            else:
                self.win.logger.log("文件保存失败。", "ERROR")

    # =========================
    # 显示与包围盒
    # =========================
    def on_bbox_toggled(self, state):
        self.win.controller.toggle_bbox(bool(state))

    def on_model_axes_toggled(self, state):
        self.win.model.show_model_axes = bool(state)
        # 强制刷新
        self.win.view3d.render_scene(self.win.model, recenter=False)

    def on_point_size_changed(self, value):
        self.win.controller.set_point_size(float(value))

    # =========================
    # 点云处理
    # =========================
    def on_use_pcd_toggled(self, state):
        num_points = self.win.spin_sample_points.value()
        self.win.controller.toggle_use_sampled_pcd(bool(state), num_points=num_points)
        status = "启用" if state else "禁用"
        self.win.logger.log(f"点云模式已{status} (采样数: {num_points})")

    def on_apply_downsample(self):
        voxel_size = self.win.spin_down_voxel.value()
        self.win.controller.apply_voxel_downsample(voxel_size)
        self.win.logger.log(f"应用体素降采样 (Voxel: {voxel_size})", "INFO")

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
        return (1.0, 1.0, 1.0)

    def on_show_normals_toggled(self, state):
        self.win.controller.toggle_show_normals(bool(state))

    def on_normal_params_changed(self, *args):
        length = self.win.spin_normal_len.value()
        step = self.win.spin_normal_step.value()
        color = self._get_normal_color_from_combo()
        self.win.controller.set_normal_params(length=length, step=step, color=color)

    # =========================
    # 视点切换
    # =========================
    def change_view(self, view_name: str):
        self.win.controller.set_view(view_name)
        self.win.logger.log(f"切换视角: {view_name}")

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
        elif text == "白":
            return (1.0, 1.0, 1.0)
        return None  # 自动

    def on_update_frustum_clicked(self):
        fov = self.win.spin_cam_fov.value()
        near = self.win.spin_cam_near.value()
        far = self.win.spin_cam_far.value()
        best = self.win.spin_cam_best.value()

        pos = np.array([
            self.win.spin_cam_posx.value(),
            self.win.spin_cam_posy.value(),
            self.win.spin_cam_posz.value(),
        ])
        direction = np.array([
            self.win.spin_cam_dirx.value(),
            self.win.spin_cam_diry.value(),
            self.win.spin_cam_dirz.value(),
        ])

        self.win.camera_controller.set_intrinsics(fov_deg=fov, near=near, far=far, best_distance=best)
        self.win.camera_controller.set_pose(pos=pos, direction=direction)
        self.win.camera_controller.update_frustum()
        self.win.logger.log("相机参数已更新，视锥重绘。", "INFO")

    def on_scan_once_clicked(self):
        color = self._get_scan_color_from_combo()
        use_occ = self.win.cb_use_occlusion.isChecked()
        self.win.camera_controller.scan_once(color=color, use_occlusion=use_occ)
        self.refresh_view_table()
        self.win.logger.log("执行单次扫描完成。", "SUCCESS")

    # =========================
    # 视点 / 扫描帧表格
    # =========================
    def refresh_view_table(self):
        win = self.win
        views = win.camera_controller.list_views()
        win.view_table.blockSignals(True)
        win.view_table.setRowCount(len(views))

        from PyQt5.QtCore import Qt
        for row, info in enumerate(views):
            # 名称 (不可编辑)
            item_name = QTableWidgetItem(info["name"])
            item_name.setFlags(item_name.flags() ^ 2)  # Bitwise XOR to remove IsEditable
            win.view_table.setItem(row, 0, item_name)

            # 可见性复选框 (居中)
            w = QWidget()
            l = QHBoxLayout(w)
            l.setContentsMargins(0, 0, 0, 0)

            l.setAlignment(Qt.AlignCenter)

            cb = QCheckBox()
            cb.setChecked(info["visible"])
            cb.stateChanged.connect(lambda state, r=row: self.on_view_visibility_changed(r, state))
            l.addWidget(cb)
            win.view_table.setCellWidget(row, 1, w)

            # 颜色文本
            c = info["color"]
            # 简单显示一个小方块或者文字
            item_color = QTableWidgetItem(f"{c[0]:.1f},{c[1]:.1f},{c[2]:.1f}")
            win.view_table.setItem(row, 2, item_color)

        win.view_table.blockSignals(False)

    def on_view_visibility_changed(self, row, state):
        visible = bool(state)
        self.win.camera_controller.set_view_visibility(row, visible)

    def on_delete_selected_view(self):
        win = self.win
        row = win.view_table.currentRow()
        if row < 0: return
        win.camera_controller.remove_view(row)
        self.refresh_view_table()
        self.win.logger.log(f"已删除视点 ID: {row}", "WARNING")

    def on_grab_camera_clicked(self):
        win = self.win
        win.camera_controller.grab_pose_from_current_view()

        # 更新 UI
        cam = win.camera_model
        win.spin_cam_posx.setValue(cam.position[0])
        win.spin_cam_posy.setValue(cam.position[1])
        win.spin_cam_posz.setValue(cam.position[2])
        win.spin_cam_dirx.setValue(cam.direction[0])
        win.spin_cam_diry.setValue(cam.direction[1])
        win.spin_cam_dirz.setValue(cam.direction[2])
        self.win.logger.log("已同步 3D 视图相机参数至 UI。", "INFO")

    def on_view_table_double_clicked(self, item):
        win = self.win
        row = item.row()
        win.camera_controller.go_to_view(row)

        # 顺便更新 UI 参数
        self.on_grab_camera_clicked()
        self.win.logger.log(f"相机跳转至视点 ID: {row}", "INFO")

    def on_save_views_clicked(self):
        filename, _ = QFileDialog.getSaveFileName(self.win, "保存视点列表", "", "Text (*.txt)")
        if filename:
            self.win.camera_controller.save_views_to_txt(filename)
            self.win.logger.log("视点列表已保存。", "SUCCESS")

    def on_show_only_selected_scan(self):
        win = self.win
        row = win.view_table.currentRow()
        if row < 0: return

        views = win.camera_controller.list_views()
        for i in range(len(views)):
            win.camera_controller.set_view_visibility(i, i == row)
        self.refresh_view_table()

    def on_export_selected_scan(self):
        row = self.win.view_table.currentRow()
        if row < 0: return
        filename, _ = QFileDialog.getSaveFileName(self.win, "导出选中扫描", "", "PLY (*.ply)")
        if filename:
            self.win.camera_controller.export_scan_pcd(row, filename)
            self.win.logger.log(f"扫描帧 {row} 已导出。", "SUCCESS")

    def on_export_all_scans(self):
        filename, _ = QFileDialog.getSaveFileName(self.win, "导出合并点云", "", "PLY (*.ply)")
        if filename:
            self.win.camera_controller.export_all_scans(filename)
            self.win.logger.log("所有扫描已合并导出。", "SUCCESS")

    # =========================
    # NBV 相关
    # =========================
    def on_nbv_mode_changed(self, index):
        win = self.win
        text = win.combo_nbv_mode.currentText()
        if text == "中心球壳":
            win.nbv_controller.mode = "sphere"
        else:
            win.nbv_controller.mode = "surface"

        win.nbv_controller.coverage = None
        self.win.logger.log(f"NBV 模式切换为: {win.nbv_controller.mode} (覆盖网格已重置)", "INFO")

    def on_nbv_params_changed(self, *args):
        win = self.win
        voxel = win.spin_nbv_voxel.value()
        num_cand = win.spin_nbv_candidates.value()

        win.nbv_controller.voxel_size = float(voxel)
        win.nbv_controller.num_candidates = int(num_cand)
        win.nbv_controller.coverage = None  # 重置
        self.win.logger.log("NBV 参数更新 (Voxel/Count)，覆盖网格已重置。", "INFO")

    def on_nbv_next_clicked(self):
        self.win.logger.log("正在计算下一最佳视点 (NBV)...", "INFO")
        # 由于 NBV 计算可能耗时，理想情况应放在线程中，这里简化处理
        self.win.nbv_controller.run_one_step()
        self.refresh_view_table()
        self.win.logger.log("NBV 计算完成，新视点已添加。", "SUCCESS")

    def on_nbv_curvature_weight_changed(self, value):
        self.win.nbv_controller.curvature_weight = float(value)
