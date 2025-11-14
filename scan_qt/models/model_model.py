from dataclasses import dataclass, field
import numpy as np

@dataclass
class ModelModel:
    base_geom: object = None
    current_geom: object = None
    sampled_pcd: object = None
    downsampled_pcd: object = None

    bbox_geom: object = None
    voxel_grid: object = None
    normal_lines: object = None

    show_bbox: bool = False
    show_voxel: bool = False
    show_normals: bool = False
    use_sampled_pcd: bool = False

    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    radius: float = 1.0

    point_size: float = 1.0
    default_pcd_color: np.ndarray = field(
        default_factory=lambda: np.array([0.6, 0.6, 0.6])
    )

    normal_step: int = 10
    normal_length: float = 0.01
    normal_color: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0])
    )

    voxel_size: float = 0.01
