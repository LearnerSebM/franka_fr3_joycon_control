"""
Launch replay_trajectory with the **franka_data** conda Python (or the env named by ``conda_env``).

Requires ``conda activate <conda_env>`` before ``ros2 launch`` so ``CONDA_PREFIX`` points
at that environment (same pattern as ``data_recorder.launch.py``).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *_args, **_kwargs):
    conda_env = LaunchConfiguration("conda_env").perform(context).strip()
    trajectory_h5 = os.path.expanduser(
        LaunchConfiguration("trajectory_h5").perform(context).strip()
    )
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    dynamic_factor = float(LaunchConfiguration("dynamic_factor").perform(context))
    realtime_config = LaunchConfiguration("realtime_config").perform(context)
    joint_dataset = LaunchConfiguration("joint_dataset").perform(context)
    timestamp_dataset = LaunchConfiguration("timestamp_dataset").perform(context)
    stride = int(LaunchConfiguration("stride").perform(context))
    go_to_start_ms = int(LaunchConfiguration("go_to_start_ms").perform(context))
    skip_first = LaunchConfiguration("skip_first_after_go_to_start").perform(context).lower()
    if skip_first not in ("true", "false"):
        raise ValueError(
            f"skip_first_after_go_to_start must be true or false, got {skip_first!r}"
        )

    conda_env_path = os.environ.get("CONDA_PREFIX", "").strip()
    if not conda_env_path:
        raise RuntimeError(
            "CONDA_PREFIX is not set. Activate your conda environment first "
            f"(e.g. `conda activate {conda_env}`) before running this launch file."
        )
    active_name = os.path.basename(conda_env_path.rstrip(os.sep))
    if conda_env and active_name != conda_env:
        raise RuntimeError(
            f"This launch expects conda env {conda_env!r}, but the active env is {active_name!r} "
            f"(CONDA_PREFIX={conda_env_path!r}). Run: conda activate {conda_env}"
        )

    python_executable = os.path.join(conda_env_path, "bin", "python")
    if not os.path.isfile(python_executable):
        raise FileNotFoundError(
            f"Expected conda Python at {python_executable} but it does not exist."
        )

    from ament_index_python.packages import get_package_share_directory

    share = get_package_share_directory("data_recorder")
    install_base = os.path.dirname(os.path.dirname(share))
    entry_script = os.path.join(install_base, "lib", "data_recorder", "replay_trajectory")
    entry_script = os.path.abspath(entry_script)

    if not os.path.isfile(entry_script):
        raise FileNotFoundError(
            f"replay_trajectory executable not found at {entry_script}; "
            "build the workspace (data_recorder uses lib/data_recorder per setup.cfg)."
        )

    if not trajectory_h5:
        raise RuntimeError(
            "trajectory_h5 is empty. Set launch argument trajectory_h5, e.g. "
            "trajectory_h5:=/path/to/trajectory.h5"
        )

    cmd = [
        python_executable,
        entry_script,
        "--ros-args",
        "-p", f"trajectory_h5:={trajectory_h5}",
        "-p", f"robot_ip:={robot_ip}",
        "-p", f"dynamic_factor:={dynamic_factor}",
        "-p", f"realtime_config:={realtime_config}",
        "-p", f"joint_dataset:={joint_dataset}",
        "-p", f"timestamp_dataset:={timestamp_dataset}",
        "-p", f"stride:={stride}",
        "-p", f"go_to_start_ms:={go_to_start_ms}",
        "-p", f"skip_first_after_go_to_start:={skip_first}",
    ]

    node_env = os.environ.copy()
    node_env["PYTHONUNBUFFERED"] = "1"

    return [
        ExecuteProcess(
            cmd=cmd,
            name="replay_trajectory",
            output="screen",
            env=node_env,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("conda_env", default_value="franka_data"),
            DeclareLaunchArgument("trajectory_h5", default_value=""),
            DeclareLaunchArgument("robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("dynamic_factor", default_value="0.2"),
            DeclareLaunchArgument("realtime_config", default_value="ignore"),
            DeclareLaunchArgument(
                "joint_dataset",
                default_value="observation/robot_state/joint_positions",
            ),
            DeclareLaunchArgument(
                "timestamp_dataset",
                default_value="observation/timestamp/robot_state",
            ),
            DeclareLaunchArgument("stride", default_value="1"),
            DeclareLaunchArgument("go_to_start_ms", default_value="0"),
            DeclareLaunchArgument("skip_first_after_go_to_start", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
