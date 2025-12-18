import open3d as o3d
import numpy as np
import time
import os
import copy
from see_planner import SEEPlanner
from simulation_env import VirtualScanner
import viz_utils

# --- 全局设置 ---
MODEL_PATH = "D:/Viewpoint Planning/Auto_Scan/scan_qt/scan_qt/resources/test1模型/test_1.ply"
TARGET_COVERAGE = 98
MAX_STEPS = 40


def robust_load_model(path):
    """
    健壮的模型加载与归一化
    """
    if not os.path.exists(path):
        print("[System] Generating synthetic Stanford Bunny (Torus)...")
        mesh = o3d.geometry.TriangleMesh.create_torus(torus_radius=0.3, tube_radius=0.1)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(path, mesh)
    else:
        print(f"[System] Loading {path}...")
        mesh = o3d.io.read_triangle_mesh(path)

    # 强制归一化到单位球内，最大尺寸 0.8m
    # 这样可以保证 sensor params (min=0.4, max=0.7) 永远适用
    center = mesh.get_center()
    mesh.translate(-center)

    bbox = mesh.get_max_bound() - mesh.get_min_bound()
    max_dim = np.max(bbox)
    if max_dim < 1e-4: raise ValueError("Model is empty or too small!")

    scale_factor = 0.8 / max_dim
    mesh.scale(scale_factor, center=[0, 0, 0])
    print(f"[System] Model normalized. Scale factor: {scale_factor:.4f}")

    return mesh


def compute_accurate_coverage(scan_pcd, gt_pcd_tree, gt_points, threshold=0.03):
    """
    基于 GT -> Scan 的单向距离查询。
    只要 GT 点附近 threshold 范围内有 Scan 点，就算覆盖。
    """
    if len(scan_pcd.points) == 0: return 0.0

    # 这是一个反向思维：我们不查 scan 覆盖了谁，我们查 gt 点离 scan 有多近
    # 这里的输入反转一下效率更高：
    # tree 应该是 scan_pcd 的 tree
    scan_tree = o3d.geometry.KDTreeFlann(scan_pcd)

    covered_count = 0
    # 随机采样 GT 点进行评估 (全量计算太慢)
    check_indices = np.random.choice(len(gt_points), min(5000, len(gt_points)), replace=False)

    for idx in check_indices:
        pt = gt_points[idx]
        [k, _, dist_sq] = scan_tree.search_knn_vector_3d(pt, 1)
        if k > 0 and dist_sq[0] < threshold ** 2:
            covered_count += 1

    return (covered_count / len(check_indices)) * 100.0


def main():
    # 1. 初始化环境
    try:
        mesh = robust_load_model(MODEL_PATH)
    except Exception as e:
        print(f"[Error] Failed to load model: {e}")
        return

    # 生成真值点云
    gt_pcd = mesh.sample_points_uniformly(number_of_points=20000)
    gt_points = np.asarray(gt_pcd.points)

    # 初始化扫描仪 (使用高模作为场景)
    scanner = VirtualScanner(
        mesh
    )
    scanner.scene_mesh = mesh
    scanner.scene_pcd = mesh.sample_points_uniformly(number_of_points=100000)
    scanner.scene_tree = o3d.geometry.KDTreeFlann(scanner.scene_pcd)

    # 初始化规划器
    planner = SEEPlanner({'min_dist': 0.45, 'max_dist': 0.75, 'fov': 60})

    # 可视化配置
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="SEE Robust Planner", width=1280, height=800)

    # 绘制参考线框
    mesh_wire = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    mesh_wire.paint_uniform_color([0.8, 0.8, 0.8])
    vis.add_geometry(mesh_wire)

    # 动态几何体
    vis_traj = o3d.geometry.LineSet()
    vis_heatmap = o3d.geometry.PointCloud()
    vis.add_geometry(vis_traj)
    vis.add_geometry(vis_heatmap)

    # 运行状态
    curr_pos = np.array([0.0, 0.6, 0.6])  # 初始稍微偏一点
    curr_look = np.array([0.0, 0.0, 0.0])

    history_pos = []
    accumulated_pcd = o3d.geometry.PointCloud()

    print(">> > Start Scanning << < ")

    step_control = [False]

    def space_callback(vis):
        print(" -> [Input] Space pressed. Capturing next view...")
        step_control[0] = True
        return False  # 返回 False 表示不需要由 Open3D 自动更新 geometry，我们自己手动更新

    # 注册按键：空格键的 ASCII 码是 32
    vis.register_key_callback(ord(" "), space_callback)

    # 初始视角设置 (可选)
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)

    print(">> > Start Scanning (Press SPACE to capture) << < ")

    for step in range(1, MAX_STEPS + 1):
        history_pos.append(curr_pos)

        print(f"Waiting for input... (Step {step})")
        while not step_control[0]:
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.01)  # 防止 CPU 占用率 100%

        # 重置开关，为下一步做准备
        step_control[0] = False

        # --- 1. 扫描 ---
        scan_cloud = scanner.capture(curr_pos, curr_look)

        # 累积点云用于可视化和覆盖率计算
        if len(scan_cloud.points) > 0:
            # 给这一帧上色
            scan_cloud.paint_uniform_color(viz_utils.get_rainbow_color(step, MAX_STEPS))

            # 融合进总点云
            accumulated_pcd += scan_cloud
            # 这里的 down_sample 只是为了防止内存爆炸，不影响规划器内部精度
            accumulated_pcd = accumulated_pcd.voxel_down_sample(0.01)

            vis.add_geometry(scan_cloud, reset_bounding_box=False)

        # --- 2. 更新规划器 ---
        planner.update_map(scan_cloud, curr_pos)

        # --- 3. 计算覆盖率 ---
        cov = compute_accurate_coverage(accumulated_pcd, None, gt_points, threshold=0.035)

        bar = viz_utils.get_progress_bar(cov)
        print(f"{bar} Step: {step} | Cov: {cov:.2f}% | Views: {len(history_pos)}")

        if cov >= TARGET_COVERAGE:
            print(">>> Exploration Complete! <<<")
            break

        # --- 4. 决策下一视点 ---
        next_view, candidates = planner.get_next_view(curr_pos)

        if next_view is None or next_view['score'] == -np.inf:
            print("[Warning] No valid view found. Terminating.")
            break

        # --- 5. 可视化刷新 ---
        # 轨迹
        if len(history_pos) > 1:
            vis.remove_geometry(vis_traj, reset_bounding_box=False)
            vis_traj = viz_utils.create_trajectory_lines(history_pos)
            vis.add_geometry(vis_traj, reset_bounding_box=False)

        # 候选热力图
        vis.remove_geometry(vis_heatmap, reset_bounding_box=False)
        if candidates:
            vis_heatmap = viz_utils.create_heatmap_cloud(candidates)
            vis.add_geometry(vis_heatmap, reset_bounding_box=False)

        # 相机位置
        cam_mesh, cam_axis = viz_utils.create_camera_actor(curr_pos, curr_look, scale=0.05)
        vis.add_geometry(cam_mesh, reset_bounding_box=False)
        vis.add_geometry(cam_axis, reset_bounding_box=False)

        vis.poll_events()
        vis.update_renderer()

        # 状态更新
        curr_pos = next_view['position']
        curr_look = next_view['look_at']

        time.sleep(0.01)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
