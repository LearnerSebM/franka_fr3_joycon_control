#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

############################################################################
# Parameters:
# controller_name: Name of the controller to spawn (required, no default)
# robot_config_file: Path to the robot configuration file to load
#                   (default: franka.config.yaml in franka_bringup/config)
#
# The example.launch.py launch file provides a flexible and unified interface
# for launching Franka Robotics example controllers via the 'controller_name'
# parameter, such as 'elbow_example_controller'.
# Example:
# ros2 launch franka_bringup example.launch.py controller_name:=elbow_example_controller
#
# This script "includes" franka.launch.py to declare core component nodes,
# including: robot_state_publisher, ros2_control_node, joint_state_publisher,
# joint_state_broadcaster, franka_robot_state_broadcaster, and optionally
# franka_gripper and rviz, with support for namespaced and non-namespaced
# environments as defined in franka.config.yaml. RViz is launched if
# 'use_rviz' is set to true in the configuration file.
#
# The default robot_config_file is franka.config.yaml in the
# franka_bringup/config directory. See that file for its own documentation.
#
# This approach improves upon the earlier individual launch scripts, which
# varied in structure and lacked namespace support, offering a more consistent
# and maintainable solution. While some may favor the older scripts for their
# specific use cases, example.launch.py enhances scalability and ease of use
# for a wide range of Franka Robotics applications.
#
# Ensure the specified  controller_name matches a controller defined in
#  controllers.yaml to avoid runtime errors.
############################################################################


import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Add the path to the `utils` folder
package_share = get_package_share_directory('joycon_control_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'joycon_control_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml  # noqa: E402

# Iterates over the uncommented lines in file specified by the robot_config_file parameter.
# "Includes" franka.launch.py for each active (uncommented) Robot.
# That file is well documented.
# The function also checks if the 'use_rviz' parameter is set to true in the YAML file.
# If so, it includes a node for RViz to visualize the robot's state.
# The function returns a list of nodes to be launched.


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    configs = load_yaml(config_file)
    nodes = []
    for item_name, config in configs.items():
        namespace = config['namespace']


        # 使用 conda 环境中的 Python 来运行 joycon_publisher
        # 获取 conda 环境的 Python 路径
        conda_env_path = os.environ.get('CONDA_PREFIX', '')
        if conda_env_path:
            python_executable = os.path.join(conda_env_path, 'bin', 'python')
        else:
            # 如果没有激活 conda 环境，尝试使用常见的 conda 环境路径
            conda_base = os.environ.get('CONDA_DEFAULT_ENV', 'joycon')
            possible_paths = [
                os.path.expanduser(f'~/Tools/miniforge3/envs/{conda_base}/bin/python'),
                os.path.expanduser(f'~/anaconda3/envs/{conda_base}/bin/python'),
                os.path.expanduser(f'~/miniconda3/envs/{conda_base}/bin/python'),
            ]
            python_executable = None
            for path in possible_paths:
                if os.path.exists(path):
                    python_executable = path
                    break
            
            if python_executable is None:
                # 如果找不到，回退到系统 Python，但会显示警告
                python_executable = '/usr/bin/python3'
                print(f"警告: 未找到 conda 环境，使用系统 Python: {python_executable}")
        
        # 获取 joycon_publisher entry point 脚本路径
        # entry_points 脚本在 install/lib/package_name/ 目录下
        # 使用 ament_index 获取包路径，然后构建脚本路径
        from ament_index_python.packages import get_package_share_directory
        joycon_wrapper_share = get_package_share_directory('joycon_wrapper')
        # 从 share/joycon_wrapper 回到 install 目录，然后到 lib/joycon_wrapper/joycon_publisher
        install_base = os.path.dirname(os.path.dirname(joycon_wrapper_share))
        entry_point_script = os.path.join(
            install_base, 'lib', 'joycon_wrapper', 'joycon_publisher'
        )
        entry_point_script = os.path.abspath(entry_point_script)
        
        # 如果脚本存在，使用脚本；否则使用模块方式
        if os.path.exists(entry_point_script):
            cmd = [python_executable, entry_point_script]
        else:
            # 使用 Python 模块方式运行
            cmd = [python_executable, '-m', 'joycon_wrapper.joycon_publisher']
        
        # 添加 ROS2 命名空间参数
        # 确保命名空间以 / 开头（完全限定名称）
        ns = namespace if namespace.startswith('/') else f'/{namespace}'
        cmd.extend(['--ros-args', '-r', f'__ns:={ns}', '-r', '__node:=joycon_publisher'])
        
        nodes.append(
            ExecuteProcess(
                cmd=cmd,
                name='joycon_publisher',
                output='screen',
                env=os.environ.copy(),
            )
        )
        
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('joycon_control_bringup'), 'launch', 'franka.launch.py'
                    ])
                ),
                launch_arguments={
                    'arm_id': str(config['arm_id']),
                    'arm_prefix': str(config['arm_prefix']),
                    'namespace': str(namespace),
                    'urdf_file': str(config['urdf_file']),
                    'robot_ip': str(config['robot_ip']),
                    'load_gripper': str(config['load_gripper']),
                    'use_fake_hardware': str(config['use_fake_hardware']),
                    'fake_sensor_commands': str(config['fake_sensor_commands']),
                    'joint_state_rate': str(config['joint_state_rate']),
                }.items(),
            )
        )


        # spawn robot_reset_controller
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=['robot_reset_controller', '--controller-manager-timeout', '30'],
                parameters=[PathJoinSubstitution([
                    FindPackageShare('joycon_control_bringup'), 'config', "controllers.yaml",
                ])],
                output='screen',
            )
        )
        # witchController node
        nodes.append(
            Node(
                package='joycon_control_bringup',
                executable='switch_controller',
                namespace=namespace,
                parameters=[{
                    'namespace': namespace,
                    'old_controller': 'robot_reset_controller',
                    'new_controller': controller_name,
                    'check_time': 0.5,
                }],
                output='screen',
            )
        )

    if any(str(config.get('use_rviz', 'false')).lower() == 'true' for config in configs.values()):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['--display-config', PathJoinSubstitution([
                    FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'
                ])],
                output='screen',
            )
        )
    return nodes

# The generate_launch_description function is the entry point (like "main")
# It is called by the ROS 2 launch system when the launch file is executed.
# via: ros2 launch franka_bringup example.launch.py ARGS...
# This function must return a LaunchDescription object containing nodes to be launched.
# it calls the generate_robot_nodes function to get the list of nodes to be launched.


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('joycon_control_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            'controller_name',
            description='Name of the controller to spawn (required, no default)',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
