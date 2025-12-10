# scan_qt/models/camera_model.py
import numpy as np


class CameraModel:
    """
    扫描仪/相机的参数模型，不依赖 Open3D/Qt。
    """

    def __init__(self):
        # FOV: 视场角（度数）
        self.fov_deg = 30

        # DOF: 近远裁剪平面
        self.near = 430
        self.far = 680

        # 扫描最佳距离
        self.best_distance = 530

        # “成像分辨率”：用于采样/做简单深度缓冲时的宽高（这里先占位）
        self.image_width = 480
        self.image_height = 480

        # 位姿（世界坐标）
        self.position = np.array([0.0, 0.0, 1.0], dtype=float)
        self.direction = np.array([0.0, 0.0, -1.0], dtype=float)  # 朝向
        self.up = np.array([0.0, 1.0, 0.0], dtype=float)          # 上方向

    def get_normalized_axes(self):
        """
        返回正规化的 front / up / right 三个单位向量。
        front 表示相机看到的方向。
        """
        front = self.direction.astype(float)
        front_norm = np.linalg.norm(front) + 1e-12
        front = front / front_norm

        up = self.up.astype(float)
        up = up - np.dot(up, front) * front
        up_norm = np.linalg.norm(up) + 1e-12
        up = up / up_norm

        right = np.cross(front, up)
        r_norm = np.linalg.norm(right) + 1e-12
        right = right / r_norm

        return front, up, right
