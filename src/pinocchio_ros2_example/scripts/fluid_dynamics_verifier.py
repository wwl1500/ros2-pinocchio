#!/usr/bin/env python3
import math
from typing import Dict, Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import pinocchio as pin

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import ParameterType


class FluidDynamicsVerifier(Node):
    """
    基于 Pinocchio 的流体动力验证节点。

    - 订阅关节状态 `/joint_states` 及给定的外力矩话题（默认 `/tau_ext`）。
    - 在本地重建 IEEE 6907522 中的流体力模型，计算空间外力，
      再通过 RNEA (带/不带外力) 求得 τ_with 与 τ_no。
    - 比较 `tau_ext_calc = τ_with - τ_no` 与话题输入的 `tau_ext`，输出残差。
    - 同时验证动力学方程 `M(q)q̈ + C(q, q̇)q̇ + g(q) = τ_with - tau_ext`
      的数值误差。
    """

    def __init__(self) -> None:
        super().__init__('fluid_dynamics_verifier')

        self.declare_parameter('tau_topic', '/tau_ext')

        # URDF 与基座类型
        self.urdf_path = self.declare_parameter('urdf_path', '').get_parameter_value().string_value
        use_free_flyer = self.declare_parameter('free_flyer', False).get_parameter_value().bool_value
        base_joint = pin.JointModelFreeFlyer() if use_free_flyer else None

        if self.urdf_path == '':
            raise RuntimeError('参数 urdf_path 为空，请指定机器人 URDF 路径。')

        if base_joint is None:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
        else:
            self.model = pin.buildModelFromUrdf(self.urdf_path, base_joint)
        self.data = self.model.createData()
        self.nq, self.nv = self.model.nq, self.model.nv
        self.q = pin.neutral(self.model)
        self.v = np.zeros(self.nv)
        self.a = np.zeros(self.nv)

        # 受流体影响的帧
        frames_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        raw_frames_param = self.declare_parameter(
            'fluid_frames', ['link_1', 'link_2'], descriptor=frames_desc
        ).get_parameter_value()
        if raw_frames_param.string_array_value is not None and len(raw_frames_param.string_array_value) > 0:
            self.fluid_frames: List[str] = list(raw_frames_param.string_array_value)
        else:
            self.fluid_frames = []
        if len(self.fluid_frames) == 0:
            self.get_logger().warn('fluid_frames 为空，将不会进行计算。')
        self.frame_ids = [self.model.getFrameId(name) for name in self.fluid_frames]

        # 全局参数
        self.rho = self.declare_parameter('rho', 1000.0).value
        current_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        raw_current_param = self.declare_parameter(
            'v_current_world', [0.0, 0.0, 0.0], descriptor=current_desc
        ).get_parameter_value()
        if raw_current_param.double_array_value is not None and len(raw_current_param.double_array_value) == 3:
            self.v_current_world = np.array(raw_current_param.double_array_value, dtype=float)
        else:
            self.v_current_world = np.zeros(3)

        default_area = self.declare_parameter('default_area', 0.01).value
        default_cd_lin = self.declare_parameter('default_cd_lin', 0.0).value
        default_cd_quad = self.declare_parameter('default_cd_quad', 0.0).value
        default_cd_ang_lin = self.declare_parameter('default_cd_ang_lin', 0.1).value

        # 帧参数
        self.frame_area = {name: default_area for name in self.fluid_frames}
        self.frame_cd_lin = {name: default_cd_lin for name in self.fluid_frames}
        self.frame_cd_quad = {name: default_cd_quad for name in self.fluid_frames}
        self.frame_cd_ang_lin = {name: default_cd_ang_lin for name in self.fluid_frames}

        # 附加质量 (μ_n, μ_t, λ_1)
        self.added_mass = {
            name: {
                'mu': self.declare_parameter(f'{name}.m_u', 0.089).value,
                'mv': self.declare_parameter(f'{name}.m_v', 0.089).value,
                'mr': self.declare_parameter(f'{name}.m_r', 0.16).value,
            }
            for name in self.fluid_frames
        }

        # 线性阻尼 (c_t, c_n)
        self.lin_damp = {
            name: {
                'du': self.declare_parameter(f'{name}.d_lu', 0.4).value,
                'dv': self.declare_parameter(f'{name}.d_lv', 3.0).value,
                'dr': self.declare_parameter(f'{name}.d_lr', 0.0).value,
            }
            for name in self.fluid_frames
        }

        # 非线性阻尼（二次项）
        self.nonlin_damp = {
            name: {
                'du': self.declare_parameter(f'{name}.d_nu', 12.9).value,
                'dv': self.declare_parameter(f'{name}.d_nv', 21.3).value,
                'dr': self.declare_parameter(f'{name}.d_nr', 0.0).value,
            }
            for name in self.fluid_frames
        }

        self.dist_added = {name: np.zeros(3) for name in self.fluid_frames}
        self.dist_damp = {name: np.zeros(3) for name in self.fluid_frames}

        # 订阅话题
        tau_topic = self.get_parameter('tau_topic').get_parameter_value().string_value
        if tau_topic == '':
            tau_topic = '/tau_ext'

        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.create_subscription(Float64MultiArray, tau_topic, self._on_tau_ext, 10)

        self.latest_tau_ext: Optional[np.ndarray] = None
        self.prev_body_vel: Dict[str, Tuple[float, float, float]] = {
            name: None for name in self.fluid_frames
        }
        self.prev_time: Optional[rclpy.time.Time] = None

        self.get_logger().info(
            f'fluid_dynamics_verifier 已启动，监听 tau 话题 {tau_topic}'
        )

    def _on_tau_ext(self, msg: Float64MultiArray) -> None:
        self.latest_tau_ext = np.array(msg.data, dtype=float)

    def _on_joint_state(self, js: JointState) -> None:
        if len(js.position) == 0:
            return

        now = self.get_clock().now()
        dt = None
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt <= 0.0:
                dt = None
        self.prev_time = now

        # 映射 q, v
        n_pos = len(js.position)
        self.q[-n_pos:] = np.array(js.position[-n_pos:])
        if len(js.velocity) >= self.nv:
            self.v = np.array(js.velocity[:self.nv])
        else:
            self.v = np.zeros(self.nv)
        self.a = np.zeros(self.nv)

        pin.forwardKinematics(self.model, self.data, self.q, self.v, self.a)
        pin.updateFramePlacements(self.model, self.data)

        fext = pin.StdVec_Force(self.model.njoints, pin.Force.Zero())

        for name, fid in zip(self.fluid_frames, self.frame_ids):
            Vw = pin.getFrameVelocity(self.model, self.data, fid, pin.ReferenceFrame.WORLD)
            v_lin = np.array(Vw.linear)
            v_ang = np.array(Vw.angular)
            v_rel_lin = v_lin - self.v_current_world
            v_rel_ang = v_ang

            X_o_f = self.data.oMf[fid]
            R_of = X_o_f.rotation.T
            u_v_body = R_of @ v_rel_lin
            r_b = v_rel_ang[2] if v_rel_ang.shape[0] == 3 else 0.0

            mu = self.added_mass[name]['mu']
            mv = self.added_mass[name]['mv']
            mr = self.added_mass[name]['mr']

            dlu = self.lin_damp[name]['du']
            dlv = self.lin_damp[name]['dv']
            dlr = self.lin_damp[name]['dr']

            dnu = self.nonlin_damp[name]['du']
            dnv = self.nonlin_damp[name]['dv']
            dnr = self.nonlin_damp[name]['dr']

            u_b, v_b = u_v_body[0], u_v_body[1]
            fD_u = dlu * u_b + dnu * math.copysign(u_b * u_b, u_b)
            fD_v = dlv * v_b + dnv * math.copysign(v_b * v_b, v_b)
            fD_r = dlr * r_b + dnr * math.copysign(r_b * r_b, r_b)

            du_dt = dv_dt = dr_dt = 0.0
            prev = self.prev_body_vel[name]
            if prev is not None and dt is not None and dt > 0.0:
                du_dt = (u_b - prev[0]) / dt
                dv_dt = (v_b - prev[1]) / dt
                dr_dt = (r_b - prev[2]) / dt
            self.prev_body_vel[name] = (u_b, v_b, r_b)

            fA_u = mu * du_dt
            fA_v = mv * dv_dt
            nA_r = mr * dr_dt

            area = self.frame_area[name]
            cdq = self.frame_cd_quad[name]
            speed = np.linalg.norm(v_rel_lin) + 1e-9
            f_quad_world = -0.5 * self.rho * cdq * area * speed * v_rel_lin

            f_lin_body = np.array([fD_u + fA_u, fD_v + fA_v, 0.0])
            f_lin_world = X_o_f.rotation @ f_lin_body + f_quad_world
            n_world = -self.frame_cd_ang_lin[name] * v_rel_ang + np.array([0.0, 0.0, fD_r + nA_r])

            distA = self.dist_added[name]
            distD = self.dist_damp[name]
            f_adjust = X_o_f.rotation @ distA
            d_adjust = X_o_f.rotation @ distD

            F_world = pin.Force(f_lin_world - f_adjust - d_adjust, n_world)
            parent_joint = self.model.frames[fid].parentJoint
            F_parent = self.data.oMi[parent_joint].actInv(F_world)
            fext[parent_joint] = fext[parent_joint] + F_parent

        tau_with = pin.rnea(self.model, self.data, self.q, self.v, self.a, fext)
        tau_no = pin.rnea(self.model, self.data, self.q, self.v, self.a)
        tau_ext_calc = (tau_with - tau_no).copy()

        pin.crba(self.model, self.data, self.q)
        pin.computeCoriolisMatrix(self.model, self.data, self.q, self.v)
        pin.computeGeneralizedGravity(self.model, self.data, self.q)
        lhs = self.data.M @ self.a + self.data.C @ self.v + self.data.g
        residual = lhs - (tau_with - tau_ext_calc)

        msg = (
            f'核对结果: '
            f'||tau_ext_calc|| = {np.linalg.norm(tau_ext_calc):.3e}, '
            f'||residual|| = {np.linalg.norm(residual):.3e}'
        )

        if self.latest_tau_ext is not None:
            diff = tau_ext_calc - self.latest_tau_ext
            msg += f', 与话题差值范数 = {np.linalg.norm(diff):.3e}'

        self.get_logger().info(msg)


def main() -> None:
    rclpy.init()
    node = FluidDynamicsVerifier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


