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
# Launch file for joycon_publisher node
# 
# Parameters:
# namespace: Namespace for the joycon_publisher node (required)
#
# This launch file starts the joycon_publisher node with proper conda
# environment detection and namespace configuration.
############################################################################

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明命名空间参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace for the joycon_publisher node',
    )

    # 创建 OpaqueFunction 来生成节点
    def generate_joycon_node(context):
        namespace = LaunchConfiguration('namespace').perform(context)

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
        
        return [
            ExecuteProcess(
                cmd=cmd,
                name='joycon_publisher',
                output='screen',
                env=os.environ.copy(),
            )
        ]

    return LaunchDescription([
        namespace_arg,
        OpaqueFunction(function=generate_joycon_node),
    ])

