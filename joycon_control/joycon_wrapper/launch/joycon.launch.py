############################################################################
# Launch file for joycon_publisher node
# 
# Parameters:
# namespace: Namespace for the joycon_publisher node (required)
#
# This launch file solely starts the joycon_publisher node, with proper conda
# environment detection and namespace configuration.
############################################################################

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace for the joycon_publisher node',
    )

    def generate_joycon_node(context):
        namespace = LaunchConfiguration('namespace').perform(context)

        # use python in conda environment to run joycon_publisher
        # get python path in conda environment
        conda_env_path = os.environ.get('CONDA_PREFIX', '').strip()
        if not conda_env_path:
            raise RuntimeError(
                'CONDA_PREFIX is not set. Activate your conda environment first '
                '(e.g. `conda activate joycon`) before running this launch file.'
            )
        python_executable = os.path.join(conda_env_path, 'bin', 'python')
        if not os.path.isfile(python_executable):
            raise FileNotFoundError(
                f'Expected conda Python at {python_executable} but it does not exist.'
            )


        from ament_index_python.packages import get_package_share_directory
        joycon_wrapper_share = get_package_share_directory('joycon_wrapper')
        install_base = os.path.dirname(os.path.dirname(joycon_wrapper_share))
        entry_point_script = os.path.join(
            install_base, 'lib', 'joycon_wrapper', 'joycon_publisher'
        )
        entry_point_script = os.path.abspath(entry_point_script)
        
        if os.path.exists(entry_point_script):
            cmd = [python_executable, entry_point_script]
        else:
            cmd = [python_executable, '-m', 'joycon_wrapper.joycon_publisher']
        
        # add ros2 namespace parameter
        ns = namespace if namespace.startswith('/') else f'/{namespace}'
        cmd.extend(['--ros-args', '-r', f'__ns:={ns}', '-r', '__node:=joycon_publisher'])
        
        # to display buffered "print" logs in python scripts before Ctrl+C in the bash line 
        node_env = os.environ.copy()
        node_env['PYTHONUNBUFFERED'] = '1'
        
        return [
            ExecuteProcess(
                cmd=cmd,
                name='joycon_publisher',
                output='screen',
                env=node_env,
            )
        ]

    return LaunchDescription([
        namespace_arg,
        OpaqueFunction(function=generate_joycon_node),
    ])

