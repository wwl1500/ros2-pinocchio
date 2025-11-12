#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor


class JointStateSinePublisher(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_sine_pub')

        # 参数
        self.names = self.declare_parameter(
            'names', ['joint_1', 'joint_2'],
            descriptor=ParameterDescriptor(description='关节名称数组')
        ).get_parameter_value().string_array_value
        self.freq = self.declare_parameter('freq', 0.5).get_parameter_value().double_value  # Hz
        self.amp = self.declare_parameter('amp', 0.6).get_parameter_value().double_value    # rad
        self.rate = self.declare_parameter('rate', 50.0).get_parameter_value().double_value # Hz
        self.phase = self.declare_parameter('phase', 1.57079632679).get_parameter_value().double_value

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)
        self.get_logger().info(f'正弦关节发布器启动: names={list(self.names)}, f={self.freq}Hz, A={self.amp}rad')

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        q = []
        dq = []
        for i, _ in enumerate(self.names):
            phi = i * self.phase
            pos = self.amp * math.sin(2.0 * math.pi * self.freq * t + phi)
            vel = 2.0 * math.pi * self.freq * self.amp * math.cos(2.0 * math.pi * self.freq * t + phi)
            q.append(pos)
            dq.append(vel)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.names)
        msg.position = q
        msg.velocity = dq
        msg.effort = [0.0] * len(self.names)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStateSinePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



