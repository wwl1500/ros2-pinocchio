import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('pinocchio_ros2_example')
    default_urdf = os.path.join(pkg_share, 'models', 'two_link.urdf')
    default_params = os.path.join(pkg_share, 'config', 'fluid_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='机器人 URDF 的绝对路径。默认使用示例模型 two_link.urdf。',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML 参数文件路径（将覆盖同名参数）。',
        ),
        DeclareLaunchArgument(
            'fluid_frames',
            default_value="['link_1','link_2']",
            description="受流体作用的帧列表，字符串形式（如 ['link_1','link_2'] 或 link_1,link_2）。",
        ),
        DeclareLaunchArgument(
            'free_flyer',
            default_value='false',
            description='是否使用自由基座（Pinocchio JointModelFreeFlyer）。',
        ),
        DeclareLaunchArgument(
            'v_current_world',
            default_value='[0.0, 0.0, 0.0]',
            description='世界系洋流速度 [vx, vy, vz]。',
        ),
        DeclareLaunchArgument(
            'start_sine_pub',
            default_value='true',
            description='是否启动正弦关节发布器（产生非零速度）。',
        ),
        Node(
            package='pinocchio_ros2_example',
            executable='fluid_dynamics_cpp_node',
            name='fluid_dynamics_cpp',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'urdf_path': LaunchConfiguration('urdf_path'),
                    'free_flyer': LaunchConfiguration('free_flyer'),
                    'fluid_frames': LaunchConfiguration('fluid_frames'),
                    'v_current_world': LaunchConfiguration('v_current_world'),
                },
            ],
        ),
        Node(
            package='pinocchio_ros2_example',
            executable='joint_state_sine_pub.py',
            name='joint_state_sine_pub',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_sine_pub')),
            parameters=[{
                'names': ParameterValue(["joint_1", "joint_2"]),
                'freq': 0.5,
                'amp': 0.5,
                'rate': 50.0,
                'phase': 1.57079632679,
            }],
        ),
    ])

