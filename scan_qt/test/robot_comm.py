# scan_qt/test/robot_comm.py
import time
from typing import Dict, Optional, Tuple, Union, List
from types import SimpleNamespace
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# --- 配置区域 ---
class Frames:
    WORLD = "world"
    BASE = "base"
    TURNTABLE = "turntable"
    SCANNER = "scanner"
    OBJECT = "object"


DEFAULT_PATHS = {
    Frames.BASE: "/UR5",
    Frames.TURNTABLE: "/turtle_joint",
    Frames.OBJECT: "/receiver",
    Frames.SCANNER: "/UR5/scan",
    Frames.WORLD: "world"
}


class RobotComm:
    def __init__(self, host: str = "localhost", port: int = 23000, start_sim: bool = False, verbose: bool = True):
        self.verbose = verbose
        if self.verbose: print(f"[RobotComm] Connecting to {host}:{port}...")

        # 1. 连接 API
        self.client = RemoteAPIClient(host=host, port=port)
        self.sim = self.client.require("sim")
        self.simIK = self.client.require("simIK")
        print(f"ZMQ Client ID: {id(self.client)}")

        # 2. 初始化句柄 (全部作为成员属性)
        self.handles = self._init_handles()

        # 3. 仿真状态
        self.sync_mode = False
        if start_sim:
            self.start()

    def _init_handles(self) -> SimpleNamespace:
        """一次性获取并缓存所有句柄"""
        h = SimpleNamespace()

        # 基础组件
        h.base = self.sim.getObject(DEFAULT_PATHS[Frames.BASE])
        h.turntable = self.sim.getObject(DEFAULT_PATHS[Frames.TURNTABLE])
        h.receiver = self.sim.getObject(DEFAULT_PATHS[Frames.OBJECT])
        h.scan = self.sim.getObject(DEFAULT_PATHS[Frames.SCANNER])
        h.world = self.sim.handle_world

        h.target = self.sim.getObject("/target")
        h.tip = self.sim.getObject("/UR5/tip")

        # UR5 关节组 (tuple)
        joints = []
        for i in range(1, 7):
            joints.append(self.sim.getObject(f"/UR5/joint{i}"))
        h.ur5_joints = tuple(joints)

        if self.verbose: print(f"[RobotComm] Handles loaded. UR5 joints: {len(h.ur5_joints)}")
        return h

    # ---------------------------------------------------------
    # 1. 仿真流程控制 (Simulation Control)
    # ---------------------------------------------------------

    def start(self, sync: bool = True):
        self.sim.setStepping(sync)
        self.sim.startSimulation()
        self.sync_mode = sync
        if self.verbose: print(f"[Sim] Started (Sync={sync})")

    def stop(self):
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.05)
        if self.verbose: print("[Sim] Stopped")

    def step(self, wait: bool = True):
        """推进仿真一步 (仅同步模式有效)"""
        if self.sync_mode:
            self.client.step(wait=wait)

    # ---------------------------------------------------------
    # 2. 机器人与转台控制 (Motion Control)
    # ---------------------------------------------------------

    def set_ur5_angles(self, angles: Tuple[float, ...], instant: bool = False):
        """
        设置 UR5 6个关节角度。
        :param instant: False(默认)=物理驱动(Dynamics), True=强制瞬移(Kinematics)
        """
        if len(angles) != 6: raise ValueError("UR5 requires 6 joint angles.")

        func = self.sim.setJointPosition if instant else self.sim.setJointTargetPosition
        for handle, angle in zip(self.handles.ur5_joints, angles):
            func(handle, angle)

    def get_ur5_angles(self) -> Tuple[float, ...]:
        return tuple(self.sim.getJointPosition(h) for h in self.handles.ur5_joints)

    def set_turntable_angle(self, angle: float, instant: bool = False):
        """设置转台角度"""
        func = self.sim.setJointPosition if instant else self.sim.setJointTargetPosition
        func(self.handles.turntable, angle)

    def get_turntable_angle(self) -> float:
        return self.sim.getJointPosition(self.handles.turntable)

    # ---------------------------------------------------------
    # 3. 空间感知 (Perception & TF)
    # ---------------------------------------------------------

    def get_pose(self, target: str, relative_to: str = Frames.WORLD) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
        """
        通用获取位姿接口。
        :param target: 目标名称 (e.g., Frames.SCANNER)
        :param relative_to: 参考系名称 (e.g., Frames.OBJECT)
        :return: (position_xyz, quaternion_xyzw)
        """
        h_obj = self._resolve_frame(target)
        h_rel = self._resolve_frame(relative_to)

        # CoppeliaSim 返回 [x,y,z, qx,qy,qz,qw]
        pose = self.sim.getObjectPose(h_obj, h_rel)
        return tuple(pose[0:3]), tuple(pose[3:7])

    def _resolve_frame(self, name: str) -> int:
        """内部辅助：将字符串名称映射为句柄"""
        mapping = {
            Frames.WORLD: self.handles.world,
            Frames.BASE: self.handles.base,
            Frames.TURNTABLE: self.handles.turntable,
            Frames.SCANNER: self.handles.scan,
            Frames.OBJECT: self.handles.receiver
        }
        return mapping.get(name, self.handles.world)

    def get_handle_position(self, handle: int, relative_to_handle: int = None) -> tuple[float]:
        """
        直接根据整数句柄获取位置
        :param relative_to_handle
        :return: [x, y, z]
        """
        ref = relative_to_handle if relative_to_handle is not None else self.sim.handle_world
        return self.sim.getObjectPosition(handle, ref)

    def get_handle_quaternion(self, handle: int, relative_to_handle: int = None) -> tuple[float]:
        """
        直接根据整数句柄获取旋转 (四元数)
        :return: [x, y, z, w]
        """
        ref = relative_to_handle if relative_to_handle is not None else self.sim.handle_world
        return self.sim.getObjectQuaternion(handle, ref)

    def get_handle_orientation(self, handle: int, relative_to_handle: int = None) -> tuple[float]:
        """
        直接根据整数句柄获取旋转 (欧拉角)
        :return: [alpha, beta, gamma] (弧度)
        """
        ref = relative_to_handle if relative_to_handle is not None else self.sim.handle_world
        return self.sim.getObjectOrientation(handle, ref)

    def get_sim_time(self):
        return self.sim.getSimulationTime()

    def close(self):
        pass  # ZMQ Client 自动管理资源
