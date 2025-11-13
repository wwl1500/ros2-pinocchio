#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
from typing import List, Tuple, Optional
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import ParameterType


class FluidDynamicsNode(Node):
    """
    基于 Pinocchio 的流体动力外力注入节点。
    
    参考文献：
        IEEE 6907522: "Modeling of underwater snake robots"
        https://ieeexplore.ieee.org/document/6907522
         
    实现的流体动力学模型包括：
    - 附加质量（Added Mass）：对应文献中的 μ_n, μ_t, λ_1
    - 线性阻尼（Linear Damping）：对应文献中的 c_t, c_n
    - 非线性阻尼（Nonlinear Damping）：对应文献中的二次阻力项
    - 未知洋流视为扰动（以参数形式进入，可置零）

    核心实现：
        将各受力帧的空间力矩聚合为 joint 外力向量 fext，传入 rnea 获取广义力矩贡献 tau_ext。
        实现公式：M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext
    """

    def __init__(self) -> None:
        super().__init__('fluid_dynamics_node')

        # URDF 与基座类型
        self.urdf_path = self.declare_parameter('urdf_path', '').get_parameter_value().string_value
        use_free_flyer = self.declare_parameter('free_flyer', False).get_parameter_value().bool_value
        base_joint = pin.JointModelFreeFlyer() if use_free_flyer else None

        # 构建模型/数据
        if self.urdf_path == '':
            raise RuntimeError('参数 urdf_path 为空。请通过参数指定机器人 URDF 路径。')
        if base_joint is None:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
        else:
            self.model = pin.buildModelFromUrdf(self.urdf_path, base_joint)
        self.data = self.model.createData()
        self.nq, self.nv = self.model.nq, self.model.nv
        self.q = pin.neutral(self.model)
        self.v = np.zeros(self.nv)
        self.a = np.zeros(self.nv)

        # 配置：受流体影响的 link/frame 名称
        # 兼容两种传参：字符串列表或字符串（YAML/JSON 形式）
        import os
        try:
            frames_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
            # 提供示例默认值，便于直接运行 two_link.urdf
            raw_frames_param = self.declare_parameter('fluid_frames', ['link_1', 'link_2'], descriptor=frames_desc).get_parameter_value()
            if raw_frames_param.string_array_value is not None and len(raw_frames_param.string_array_value) > 0:
                raw_frames = list(raw_frames_param.string_array_value)
            elif raw_frames_param.string_value:
                raw_frames = raw_frames_param.string_value
            else:
                raw_frames = []
        except Exception:
            # 参数声明失败时兼容运行（例如 CLI 传型与声明型不一致）
            raw_frames = os.environ.get('FLUID_FRAMES', "['link_1','link_2']")
        if isinstance(raw_frames, list) and all(isinstance(s, str) for s in raw_frames):
            self.fluid_frames: List[str] = raw_frames
        elif isinstance(raw_frames, str) and raw_frames:
            import ast
            try:
                parsed = ast.literal_eval(raw_frames)
                self.fluid_frames = list(parsed) if isinstance(parsed, (list, tuple)) else [raw_frames]
            except Exception:
                self.fluid_frames = [s.strip() for s in raw_frames.split(',') if s.strip()]
        else:
            self.fluid_frames = []
        if len(self.fluid_frames) == 0:
            self.get_logger().warn('未配置 fluid_frames，将不施加任何流体外力。')
        self.frame_ids = [self.model.getFrameId(name) for name in self.fluid_frames] if self.fluid_frames else []

        # 物理参数（全局）
        # 介质密度（用于二次阻力；若只用线性阻尼可忽略）
        self.rho = self.declare_parameter('rho', 1000.0).get_parameter_value().double_value
        current_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        raw_current_param = self.declare_parameter('v_current_world', [0.0, 0.0, 0.0], descriptor=current_desc).get_parameter_value()
        if raw_current_param.double_array_value is not None and len(raw_current_param.double_array_value) > 0:
            raw_current = list(raw_current_param.double_array_value)
        else:
            raw_current = raw_current_param.string_value
        if isinstance(raw_current, list) and len(raw_current) > 0:
            self.v_current_world = np.array(raw_current, dtype=float)
        elif isinstance(raw_current, str):
            import ast
            try:
                self.v_current_world = np.array(ast.literal_eval(raw_current), dtype=float)
            except Exception:
                parts = [p.strip() for p in raw_current.split(',')]
                self.v_current_world = np.array([float(x) for x in parts], dtype=float)
        else:
            self.v_current_world = np.zeros(3)

        # 每个帧的几何面积（用于二次阻力），与阻力系数
        default_area = self.declare_parameter('default_area', 0.01).get_parameter_value().double_value
        default_cd_lin = self.declare_parameter('default_cd_lin', 0.0).get_parameter_value().double_value
        default_cd_quad = self.declare_parameter('default_cd_quad', 0.0).get_parameter_value().double_value
        default_cd_ang_lin = self.declare_parameter('default_cd_ang_lin', 0.1).get_parameter_value().double_value

        # 帧参数映射（可通过参数服务覆盖）
        self.frame_area = {name: default_area for name in self.fluid_frames}
        self.frame_cd_lin = {name: default_cd_lin for name in self.fluid_frames}
        self.frame_cd_quad = {name: default_cd_quad for name in self.fluid_frames}
        self.frame_cd_ang_lin = {name: default_cd_ang_lin for name in self.fluid_frames}

        # 附加质量参数（IEEE 6907522 中的 μ_n, μ_t, λ_1），按帧配置（简单对角）
        # 参考文献中的附加质量系数
        self.added_mass = {
            name: {
                'mu': self.declare_parameter(f'{name}.m_u', 0.089).value,  # 沿 u 方向附加质量 (kg)
                'mv': self.declare_parameter(f'{name}.m_v', 0.089).value,  # 沿 v 方向附加质量 (kg)
                'mr': self.declare_parameter(f'{name}.m_r', 0.16).value,  # 绕 z 轴附加转动惯量 (kg·m²)
            }
            for name in self.fluid_frames
        }

        # 线性阻尼（IEEE 6907522 中的 c_t, c_n）
        self.lin_damp = {
            name: {
                'du': self.declare_parameter(f'{name}.d_lu', 0.4).value,   # u 方向线性阻尼 (N·s/m)
                'dv': self.declare_parameter(f'{name}.d_lv', 3.0).value,   # v 方向线性阻尼 (N·s/m)
                'dr': self.declare_parameter(f'{name}.d_lr', 0.0).value,  # 旋转线性阻尼 (N·m·s/rad)
            }
            for name in self.fluid_frames
        }
        # 非线性阻尼（IEEE 6907522 中的二次阻力项）
        self.nonlin_damp = {
            name: {
                'du': self.declare_parameter(f'{name}.d_nu', 12.9).value,  # u 方向非线性阻尼 (N·s²/m²)
                'dv': self.declare_parameter(f'{name}.d_nv', 21.3).value,  # v 方向非线性阻尼 (N·s²/m²)
                'dr': self.declare_parameter(f'{name}.d_nr', 0.0).value,   # 旋转非线性阻尼 (N·m·s²/rad²)
            }
            for name in self.fluid_frames
        }

        # 扰动项（可用于补偿未知洋流）
        self.dist_added = {name: np.array([0.0, 0.0, 0.0]) for name in self.fluid_frames}
        self.dist_damp = {name: np.array([0.0, 0.0, 0.0]) for name in self.fluid_frames}

        # 订阅与发布
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.pub_tau_ext = self.create_publisher(Float64MultiArray, 'tau_ext', 10)

        self.get_logger().info('fluid_dynamics_node 已启动')

        # 记录前一时刻的体速度用于估计加速度（附加质量项）
        self.prev_body_vel: dict[str, Optional[Tuple[float, float, float]]] = {
            name: None for name in self.fluid_frames
        }
        self.prev_time = None

    # --- 工具函数 ---
    @staticmethod
    def _sign(x: np.ndarray) -> np.ndarray:
        return np.sign(x)

    def _frame_space_velocity_world(self, frame_id: int) -> pin.Motion:
        """返回世界系下帧的空间速度。"""
        return pin.getFrameVelocity(self.model, self.data, frame_id, pin.ReferenceFrame.WORLD)

    # --- 回调 ---
    def _on_joint_state(self, js: JointState) -> None:
        now = self.get_clock().now()
        dt = None
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt <= 0.0:
                dt = None
        # 简单映射（实际项目需基于 name->index 对齐）
        try:
            pos = np.array(js.position[: self.nq])
            vel = np.array(js.velocity[: self.nv]) if len(js.velocity) >= self.nv else np.zeros(self.nv)
            self.q[-len(pos) :] = pos
            self.v[-len(vel) :] = vel
        except Exception as e:
            self.get_logger().warn(f'JointState 映射失败: {e}')
            return

        # Kinematics
        pin.forwardKinematics(self.model, self.data, self.q, self.v, self.a)
        pin.updateFramePlacements(self.model, self.data)

        # 外力容器
        fext = pin.StdVec_Force(self.model.njoints, pin.Force.Zero())

        for name, fid in zip(self.fluid_frames, self.frame_ids):
            # 世界系帧速度
            Vw = self._frame_space_velocity_world(fid)
            v_lin = np.array(Vw.linear)      # m/s
            v_ang = np.array(Vw.angular)     # rad/s

            # 相对速度（减去洋流线速度；角速度不减）
            v_rel_lin = v_lin - self.v_current_world
            v_rel_ang = v_ang

            # 将世界系线速度变换到帧体坐标系，便于沿 u/v/r 使用系数
            X_o_f = self.data.oMf[fid]  # 世界->帧的 SE3
            R_of = X_o_f.rotation.T     # 世界到帧的旋转
            u_v_body = R_of @ v_rel_lin
            r_body = v_rel_ang  # 角速度在小速度下直接用世界系近似

            # IEEE 6907522: 附加质量（Added Mass）- 对应文献中的 μ_n, μ_t, λ_1
            mu = self.added_mass[name]['mu']
            mv = self.added_mass[name]['mv']
            mr = self.added_mass[name]['mr']
            distA = self.dist_added[name]

            # 线性阻尼 + 非线性阻尼（体坐标）
            dlu = self.lin_damp[name]['du']
            dlv = self.lin_damp[name]['dv']
            dlr = self.lin_damp[name]['dr']
            dnu = self.nonlin_damp[name]['du']
            dnv = self.nonlin_damp[name]['dv']
            dnr = self.nonlin_damp[name]['dr']

            u_b, v_b = u_v_body[0], u_v_body[1]
            r_b = r_body[2] if r_body.shape[0] == 3 else 0.0

            fD_u = dlu * u_b + dnu * (abs(u_b) * u_b)
            fD_v = dlv * v_b + dnv * (abs(v_b) * v_b)
            fD_r = dlr * r_b + dnr * (abs(r_b) * r_b)

            # 附加质量力（使用有限差分估计速度导数）
            du_dt = dv_dt = dr_dt = 0.0
            prev_vel = self.prev_body_vel.get(name)
            if prev_vel is not None and dt is not None:
                du_dt = (u_b - prev_vel[0]) / dt
                dv_dt = (v_b - prev_vel[1]) / dt
                dr_dt = (r_b - prev_vel[2]) / dt
            fA_u = mu * du_dt
            fA_v = mv * dv_dt
            nA_r = mr * dr_dt

            # 二次阻力（世界系，可选）
            area = self.frame_area[name]
            cdq = self.frame_cd_quad[name]
            speed = np.linalg.norm(v_rel_lin) + 1e-9
            f_quad_world = -0.5 * self.rho * cdq * area * speed * v_rel_lin

            # 将体坐标的阻尼力 fD_u, fD_v 投影到世界系 XYZ
            f_lin_body = np.array([fD_u + fA_u, fD_v + fA_v, 0.0])
            f_lin_world_from_body = X_o_f.rotation @ f_lin_body
            # 合成阻力（线/非线性 + 二次）
            force_world = f_lin_world_from_body + f_quad_world
            # 角阻尼（简单线性）
            cd_ang_lin = self.frame_cd_ang_lin[name]
            n_world = -cd_ang_lin * v_rel_ang + np.array([0.0, 0.0, fD_r + nA_r])

            # 扰动项（从体坐标转世界）
            distD_body = self.dist_damp[name]
            dist_world = X_o_f.rotation @ distD_body

            # 将附加质量扰动项叠加
            F_world = pin.Force(force_world - dist_world - X_o_f.rotation @ distA, n_world)

            # 将外力从世界系变换到该帧父关节坐标系，并累加到对应关节
            parent_joint = self.model.frames[fid].parentJoint
            X_o_parent = self.data.oMi[parent_joint]
            F_parent = X_o_parent.actInv(F_world)
            fext[parent_joint] = fext[parent_joint] + F_parent

            self.prev_body_vel[name] = (u_b, v_b, r_b)

        # 计算包含外力与不含外力的广义力矩，并求出 tau_ext
        tau_with = pin.rnea(self.model, self.data, self.q, self.v, self.a, fext)
        tau_no = pin.rnea(self.model, self.data, self.q, self.v, self.a)
        tau_ext = (tau_with - tau_no).copy()

        # 验证动力学方程 M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext
        if not getattr(self, '_verify_logged', False):
            pin.crba(self.model, self.data, self.q)
            pin.computeCoriolisMatrix(self.model, self.data, self.q, self.v)
            pin.computeGeneralizedGravity(self.model, self.data, self.q)
            lhs = self.data.M @ self.a + self.data.C @ self.v + self.data.g  # = tau_no
            rhs = tau_with  # 所需力矩 τ
            residual = lhs - (rhs - tau_ext)
            self.get_logger().info(
                f'验证公式: ||M*qdd + C*v + g - (tau - tau_ext)|| = {np.linalg.norm(residual):.3e}'
            )
            self._verify_logged = True

        # 发布用于监控/下游控制器的外力矩（不直接发到控制器，避免覆盖你的控制律）
        msg = Float64MultiArray()
        msg.data = tau_ext.tolist()
        self.pub_tau_ext.publish(msg)

        self.prev_time = now


def main() -> None:
    rclpy.init()
    node = FluidDynamicsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


