# scan_qt/robot/robot_comm.py
import math
import numpy as np

from scan_qt.coppeliasim import sim, simConst  # 按你的包结构


class RobotCommError(RuntimeError):
    pass


class RobotComm:
    """
    封装 CoppeliaSim 远程通信：
    - 建立/关闭连接
    - 获取 UR5 各关节句柄 & 转台/工件/扫描仪句柄
    - 获取世界系下的 {J},{O},{S} 变换矩阵
    - 提供通用坐标系转换接口：W/J/O/S 之间点/方向互转
    """

    FRAME_WORLD = "W"
    FRAME_TURNTABLE = "J"
    FRAME_OBJECT = "O"
    FRAME_SCANNER = "S"
    FRAME_BASE = "B"

    def __init__(self,
                 host: str = "127.0.0.1",
                 port: int = 19997,
                 connect_immediately: bool = True):
        self.host = host
        self.port = port
        self.client_id = -1

        # 对象句柄
        self.handle_turtle = None   # {J}
        self.handle_object = None   # {O}
        self.handle_scan = None     # {S}
        self.handle_base = None  # {B}

        # 可选：UR5 关节句柄（后续规划时用）
        self.ur5_joint_names = [f"joint{i}" for i in range(1, 7)]
        self.ur5_joints = []

        if connect_immediately:
            self.connect()

    # ----------------- 连接 / 断开 -----------------

    def connect(self):
        """
        建立与 CoppeliaSim 的连接并启动仿真，初始化关键句柄。
        """
        # 先断开旧连接
        sim.simxFinish(-1)

        self.client_id = sim.simxStart(
            self.host, self.port, True, True, 5000, 5
        )
        if self.client_id == -1:
            raise RobotCommError(
                f"无法连接到 CoppeliaSim ({self.host}:{self.port})"
            )

        # 尝试启动仿真（如果仿真本来就在运行，这里返回的 err 可以忽略）
        sim.simxStartSimulation(
            self.client_id, sim.simx_opmode_blocking
        )

        # 初始化关键对象句柄
        self._init_handles()

        print(f"[RobotComm] 已连接 CoppeliaSim, client_id={self.client_id}")

    def _init_handles(self):
        """
        获取所需对象的句柄：
        - UR5 joint1~6（可选）
        - turtle_joint 作为 {J}
        - receiver     作为 {O}
        - scan         作为 {S}
        - base         作为 {B}
        """
        if self.client_id == -1:
            raise RobotCommError("还未连接 CoppeliaSim，无法初始化句柄")

        # UR5 关节
        self.ur5_joints = []
        for name in self.ur5_joint_names:
            err, h = sim.simxGetObjectHandle(
                self.client_id, name, sim.simx_opmode_blocking
            )
            if err != sim.simx_return_ok:
                raise RobotCommError(f"无法获取 UR5 关节句柄: {name}, err={err}")
            self.ur5_joints.append(h)

        # 转台顶面 {J}
        err, self.handle_turtle = sim.simxGetObjectHandle(
            self.client_id, "turtle_joint", sim.simx_opmode_blocking
        )
        if err != sim.simx_return_ok:
            raise RobotCommError(
                f"无法获取转台句柄 'turtle_joint', err={err}"
            )

        # 工件 {O}
        err, self.handle_object = sim.simxGetObjectHandle(
            self.client_id, "receiver", sim.simx_opmode_blocking
        )
        if err != sim.simx_return_ok:
            raise RobotCommError(
                f"无法获取工件句柄 'receiver', err={err}"
            )

        # 扫描仪 {S}
        err, self.handle_scan = sim.simxGetObjectHandle(
            self.client_id, "scan", sim.simx_opmode_blocking
        )
        if err != sim.simx_return_ok:
            raise RobotCommError(
                f"无法获取扫描仪句柄 'scan', err={err}"
            )

        # UR5 基座 {B}
        err, self.handle_base = sim.simxGetObjectHandle(
            self.client_id, "base", sim.simx_opmode_blocking
        )
        if err != sim.simx_return_ok:
            raise RobotCommError(
                f"无法获取 UR5 基座句柄 'base', err={err}"
            )

    def is_connected(self) -> bool:
        return self.client_id != -1

    def close(self):
        """
        安全关闭仿真和连接。
        """
        if self.client_id == -1:
            return
        try:
            sim.simxStopSimulation(self.client_id, sim.simx_opmode_blocking)
        except Exception as e:
            print("[RobotComm] 停止仿真时异常:", e)
        try:
            sim.simxFinish(self.client_id)
        except Exception as e:
            print("[RobotComm] 断开连接时异常:", e)
        finally:
            print("[RobotComm] 已断开 CoppeliaSim 连接")
            self.client_id = -1

    # ----------------- 内部：Position+Orientation → T_WX -----------------

    @staticmethod
    def _euler_xyz_to_R(alpha, beta, gamma):
        """
        CoppeliaSim 的 simxGetObjectOrientation 返回的是 (alpha, beta, gamma)
        对应绕 X、Y、Z 的旋转（单位 rad）。

        通常组合方式为 R = Rz(gamma) * Ry(beta) * Rx(alpha)
        """
        ca, sa = math.cos(alpha), math.sin(alpha)
        cb, sb = math.cos(beta), math.sin(beta)
        cg, sg = math.cos(gamma), math.sin(gamma)

        Rx = np.array([[1, 0, 0],
                       [0, ca, -sa],
                       [0, sa, ca]], dtype=float)

        Ry = np.array([[cb, 0, sb],
                       [0, 1, 0],
                       [-sb, 0, cb]], dtype=float)

        Rz = np.array([[cg, -sg, 0],
                       [sg, cg, 0],
                       [0, 0, 1]], dtype=float)

        R = Rz @ Ry @ Rx
        return R

    def _get_T_WX(self, handle) -> np.ndarray:
        """
        使用 simxGetObjectPosition + simxGetObjectOrientation
        构造对象在世界系下的 4x4 齐次变换矩阵 T_WX。
        """
        if self.client_id == -1:
            raise RobotCommError("尚未连接 CoppeliaSim")

        # 位置
        err_p, pos = sim.simxGetObjectPosition(
            self.client_id, handle, -1, sim.simx_opmode_blocking
        )
        if err_p != sim.simx_return_ok:
            raise RobotCommError(
                f"simxGetObjectPosition 失败, err={err_p}"
            )
        # 欧拉角
        err_o, euler = sim.simxGetObjectOrientation(
            self.client_id, handle, -1, sim.simx_opmode_blocking
        )
        if err_o != sim.simx_return_ok:
            raise RobotCommError(
                f"simxGetObjectOrientation 失败, err={err_o}"
            )

        alpha, beta, gamma = euler  # X,Y,Z
        R = self._euler_xyz_to_R(alpha, beta, gamma)
        t = np.array(pos, dtype=float).reshape(3)

        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    # ----------------- 各坐标系的 T_W* -----------------

    def get_T_W(self) -> np.ndarray:
        """
        世界坐标系自身：恒为单位阵。
        """
        return np.eye(4, dtype=float)

    def get_T_WJ(self) -> np.ndarray:
        """
        世界系下转台顶面坐标系 {J} 的变换矩阵 T_WJ。
        """
        return self._get_T_WX(self.handle_turtle)

    def get_T_WO(self) -> np.ndarray:
        """
        世界系下工件 {O} 的变换矩阵 T_WO。
        """
        return self._get_T_WX(self.handle_object)

    def get_T_WS(self) -> np.ndarray:
        """
        世界系下扫描仪 {S} 的变换矩阵 T_WS。
        """
        return self._get_T_WX(self.handle_scan)

    def get_T_WB(self) -> np.ndarray:
        """
        世界系下 UR5 基座 {B} 的变换矩阵 T_WB。
        """
        return self._get_T_WX(self.handle_base)

    # ----------------- 通用坐标系转换 -----------------

    def _get_T_W_by_frame_name(self, frame: str) -> np.ndarray:
        frame = frame.upper()
        if frame == self.FRAME_WORLD:
            return self.get_T_W()
        elif frame == self.FRAME_TURNTABLE:
            return self.get_T_WJ()
        elif frame == self.FRAME_OBJECT:
            return self.get_T_WO()
        elif frame == self.FRAME_SCANNER:
            return self.get_T_WS()
        elif frame == self.FRAME_BASE:
            return self.get_T_WB()
        else:
            raise RobotCommError(f"未知坐标系名称: {frame}")

    def transform_point(self,
                        point: np.ndarray,
                        from_frame: str,
                        to_frame: str) -> np.ndarray:
        """
        在不同刚体坐标系之间转换点坐标（包含平移）：

        输入：
          point: (3,) in from_frame
          from_frame, to_frame in {"W","J","O","S","B"}

        输出：
          (3,) in to_frame
        """
        p = np.asarray(point, dtype=float).reshape(3)

        T_W_from = self._get_T_W_by_frame_name(from_frame)
        T_W_to = self._get_T_W_by_frame_name(to_frame)

        # from -> W: T_W_from
        # W -> to : inv(T_W_to)
        T_to_W = np.linalg.inv(T_W_to)
        T_to_from = T_to_W @ T_W_from

        p_h = np.ones(4, dtype=float)
        p_h[:3] = p
        q_h = T_to_from @ p_h
        return q_h[:3]

    def transform_direction(self,
                            direction: np.ndarray,
                            from_frame: str,
                            to_frame: str) -> np.ndarray:
        """
        在不同刚体坐标系之间转换“方向向量”（只旋转，不平移）：

        输入：
          direction: (3,) in from_frame
          from_frame, to_frame in {"W","J","O","S"}

        输出：
          (3,) in to_frame
        """
        v = np.asarray(direction, dtype=float).reshape(3)

        T_W_from = self._get_T_W_by_frame_name(from_frame)
        T_W_to = self._get_T_W_by_frame_name(to_frame)

        R_W_from = T_W_from[:3, :3]
        R_W_to = T_W_to[:3, :3]

        # v_W = R_W_from * v_from
        v_W = R_W_from @ v
        # v_to = R_to_W * v_W = R_W_to^T * v_W
        R_to_W = R_W_to.T
        v_to = R_to_W @ v_W

        return v_to

    # ----------------- 打印辅助函数 -----------------

    @staticmethod
    def _matrix_to_pos_euler(T: np.ndarray):
        """
        将 4x4 矩阵分解为 (pos, eulerXYZ)
        euler 单位 rad。
        """
        R = T[:3, :3]
        t = T[:3, 3]

        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0.0
        euler = np.array([x, y, z], dtype=float)
        return t, euler

    def print_frames_info(self):
        """
        打印 {W,J,O,S,B} 在世界系下的位姿 (pos, eulerXYZ[deg])。
        """
        frames = [
            (self.FRAME_WORLD, self.get_T_W()),
            (self.FRAME_TURNTABLE, self.get_T_WJ()),
            (self.FRAME_OBJECT, self.get_T_WO()),
            (self.FRAME_SCANNER, self.get_T_WS()),
            (self.FRAME_BASE, self.get_T_WB()),
        ]
        print("========== Frame poses (World) ==========")
        for name, T in frames:
            t, e = self._matrix_to_pos_euler(T)
            e_deg = e * 180.0 / math.pi
            print(
                f"{name}: pos = ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}), "
                f"eulerXYZ[deg] = ({e_deg[0]:.1f}, {e_deg[1]:.1f}, {e_deg[2]:.1f})"
            )
        print("=========================================")

    def print_transform_test(self,
                             point: np.ndarray,
                             from_frame: str,
                             to_frame: str):
        """
        把某个点在两个坐标系下的坐标打印出来，方便在仿真中对照。
        """
        p_from = np.asarray(point, dtype=float).reshape(3)
        p_to = self.transform_point(p_from, from_frame, to_frame)
        print(
            f"[TransformTest] {from_frame} -> {to_frame}: "
            f"p_{from_frame} = {p_from}, p_{to_frame} = {p_to}"
        )
