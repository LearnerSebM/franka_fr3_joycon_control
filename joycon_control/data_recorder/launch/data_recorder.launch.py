"""
Launch data_recorder using the **currently activated** conda env (CONDA_PREFIX).

Requires ``conda activate <env>`` before ``ros2 launch`` so h5py and other deps
match that environment; the installed entry script shebang may still be system Python.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *_args, **_kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    sample_hz = float(LaunchConfiguration("sample_hz").perform(context))
    ring_capacity = int(LaunchConfiguration("ring_capacity").perform(context))
    output_directory = LaunchConfiguration("output_directory").perform(context)
    file_prefix = LaunchConfiguration("file_prefix").perform(context)

    conda_env_path = os.environ.get("CONDA_PREFIX", "").strip()
    if not conda_env_path:
        raise RuntimeError(
            "CONDA_PREFIX is not set. Activate your conda environment first "
            "(e.g. `conda activate franka_data`) before running this launch file."
        )
    python_executable = os.path.join(conda_env_path, "bin", "python")
    if not os.path.isfile(python_executable):
        raise FileNotFoundError(
            f"Expected conda Python at {python_executable} but it does not exist."
        )

    from ament_index_python.packages import get_package_share_directory

    share = get_package_share_directory("data_recorder")
    install_base = os.path.dirname(os.path.dirname(share))
    entry_script = os.path.join(install_base, "bin", "data_recorder")
    entry_script = os.path.abspath(entry_script)

    if not os.path.exists(entry_script):
        raise FileNotFoundError(
            f"data_recorder executable not found at {entry_script}; build the workspace first."
        )

    cmd = [
        python_executable,
        entry_script,
        "--ros-args",
        "-p", f"namespace:={ns}",
        "-p", f"sample_hz:={sample_hz}",
        "-p", f"ring_capacity:={ring_capacity}",
        "-p", f"output_directory:={output_directory}",
        "-p", f"file_prefix:={file_prefix}",
    ]

    node_env = os.environ.copy()
    node_env["PYTHONUNBUFFERED"] = "1"

    return [
        ExecuteProcess(
            cmd=cmd,
            name="data_recorder",
            output="screen",
            env=node_env,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="NS_1"),
            DeclareLaunchArgument("sample_hz", default_value="15.0"),
            DeclareLaunchArgument("ring_capacity", default_value="1000"),
            DeclareLaunchArgument(
                "output_directory",
                default_value="~/joycon_recordings",
            ),
            DeclareLaunchArgument("file_prefix", default_value="recording"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
