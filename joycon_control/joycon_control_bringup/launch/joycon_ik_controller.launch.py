import sys
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

package_share = get_package_share_directory('joycon_control_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'joycon_control_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml  # noqa: E402


def generate_robot_nodes(context):
    additional_nodes = []
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)

    additional_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('joycon_control_bringup'), 'launch', 'franka_controller.launch.py'
                ])
            ),
            launch_arguments={
                'robot_config_file': robot_config_file,
                'controller_name': 'franka_joycon_controller',
            }.items(),
        )
    )

    configs = load_yaml(robot_config_file)

    for _, config in configs.items():
        robot_ip = config['robot_ip']
        namespace = config['namespace']
        load_gripper = config['load_gripper']
        use_fake_hardware = config['use_fake_hardware']
        fake_sensor_commands = config['fake_sensor_commands']
        use_rviz = config.get('use_rviz', 'false')

        # moveit ik launch
        additional_nodes.append(
          IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_fr3_moveit_config'),
                            'launch',
                            'move_group.launch.py',
                        ]
                    )
                ]
            ),
            launch_arguments={
                'robot_ip': str(robot_ip),
                'namespace': str(namespace),
                'load_gripper': str(load_gripper),
                'use_fake_hardware': str(use_fake_hardware),
                'fake_sensor_commands': str(fake_sensor_commands),
                'use_rviz': str(use_rviz),
            }.items(),
          ),
        )
    return additional_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('joycon_control_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])

