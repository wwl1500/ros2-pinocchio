#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pinocchio
import numpy as np

class PinocchioPythonNode(Node):
    def __init__(self):
        super().__init__('pinocchio_python_node')
        
        # 创建一个简单的 Pinocchio 模型
        # 在实际使用中，你可以加载 URDF 文件
        # model = pinocchio.buildModelFromUrdf("path/to/robot.urdf")
        self.model = pinocchio.Model()
        self.data = self.model.createData()
        
        self.get_logger().info('Pinocchio Python Node 启动成功')
        self.get_logger().info(f'Pinocchio 版本: {pinocchio.__version__}')
        self.get_logger().info(f'模型关节数: {self.model.njoints}')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(String, 'pinocchio_info', 10)
        
        # 创建定时器
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def compute_dynamics_example(self):
        """
        演示如何使用 Pinocchio 计算动力学方程:
        M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext
        """
        if self.model.nq == 0:
            self.get_logger().warn('模型为空，无法计算动力学。请加载URDF模型。')
            return
        
        # 1. 定义状态变量
        q = np.zeros(self.model.nq)      # 关节位置
        v = np.zeros(self.model.nv)      # 关节速度
        tau = np.zeros(self.model.nv)    # 控制力矩
        tau_ext = np.zeros(self.model.nv) # 外部力矩（如流体作用力）
        
        # 2. 计算惯性矩阵 M(q)
        # 使用 CRBA (Composite Rigid Body Algorithm) 计算惯性矩阵
        pinocchio.crba(self.model, self.data, q)
        M = self.data.M  # 惯性矩阵 M(q)
        
        # 3. 计算科氏项/离心项 C(q, q̇)q̇ 和重力项 g(q)
        # 使用 computeAllTerms 一次性计算所有项
        pinocchio.computeAllTerms(self.model, self.data, q, v)
        
        # C(q, q̇)q̇: 科氏项/离心项
        Cv = self.data.C @ v
        
        # g(q): 重力项
        g = self.data.g
        
        # 4. 添加外部力矩 τ_ext（例如流体作用力）
        # 用户可以在这里定义自己的外部力矩
        # tau_ext = self.compute_fluid_forces(q, v)  # 示例：计算流体作用力
        
        # 5. 计算完整的动力学方程
        # M(q)q̈ + C(q, q̇)q̇ + g(q) = τ + τ_ext
        # 可以重新排列为：M(q)q̈ = τ + τ_ext - C(q, q̇)q̇ - g(q)
        rhs = tau + tau_ext - Cv - g
        
        # 6. 求解加速度 q̈（如果已知力矩）
        # q̈ = M(q)^(-1) * (τ + τ_ext - C(q, q̇)q̇ - g(q))
        qdd = np.linalg.solve(M, rhs)
        
        self.get_logger().info('动力学计算完成:')
        self.get_logger().info(f'  - 惯性矩阵 M(q) 维度: {M.shape[0]} x {M.shape[1]}')
        self.get_logger().info(f'  - 科氏项 C(q,q̇)q̇ 维度: {Cv.shape[0]}')
        self.get_logger().info(f'  - 重力项 g(q) 维度: {g.shape[0]}')
        self.get_logger().info(f'  - 外部力矩 τ_ext 维度: {tau_ext.shape[0]}')
    
    def compute_inverse_dynamics(self, q, v, a, tau_ext=None):
        """
        使用 RNEA (Recursive Newton-Euler Algorithm) 计算逆动力学
        给定 q, q̇, q̈，计算所需的力矩 τ
        
        Args:
            q: 关节位置
            v: 关节速度
            a: 关节加速度
            tau_ext: 外部力矩（可选）
        
        Returns:
            tau: 所需的控制力矩
        """
        # RNEA 计算: τ = M(q)q̈ + C(q, q̇)q̇ + g(q) - τ_ext
        tau = pinocchio.rnea(self.model, self.data, q, v, a)
        
        # 如果有外部力矩，需要减去
        if tau_ext is not None and tau_ext.size == self.model.nv:
            tau -= tau_ext
        
        return tau
    
    def compute_forward_dynamics(self, q, v, tau, tau_ext=None):
        """
        使用前向动力学计算加速度
        给定 q, q̇, τ, τ_ext，计算加速度 q̈
        
        Args:
            q: 关节位置
            v: 关节速度
            tau: 控制力矩
            tau_ext: 外部力矩（可选）
        
        Returns:
            a: 关节加速度
        """
        # 1. 计算惯性矩阵
        pinocchio.crba(self.model, self.data, q)
        M = self.data.M
        
        # 2. 计算非线性项（科氏项 + 重力项）
        # 使用 computeAllTerms 计算所有项，然后组合得到非线性项
        pinocchio.computeAllTerms(self.model, self.data, q, v)
        nle = self.data.C @ v + self.data.g  # nle = C(q, q̇)q̇ + g(q)
        
        # 3. 计算总力矩
        tau_total = tau.copy()
        if tau_ext is not None and tau_ext.size == self.model.nv:
            tau_total += tau_ext
        
        # 4. 求解加速度: M(q)q̈ = τ + τ_ext - (C(q, q̇)q̇ + g(q))
        # 即: q̈ = M(q)^(-1) * (τ + τ_ext - nle)
        a = np.linalg.solve(M, tau_total - nle)
        
        return a
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Pinocchio Python 正在运行 - 版本: {pinocchio.__version__}'
        self.publisher_.publish(msg)
        
        # 演示动力学计算
        self.compute_dynamics_example()


def main(args=None):
    rclpy.init(args=args)
    node = PinocchioPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

