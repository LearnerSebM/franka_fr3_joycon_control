"""Launch data_recorder using the **currently activated** conda env (CONDA_PREFIX).

Requires ``conda activate <env>`` before ``ros2 launch`` so h5py, pyrealsense2
and cv2 match that environment; the installed entry script shebang may still
be system Python.

At launch time the user is prompted once for the language instruction that
will be stored in every episode recorded in this session. Pressing Enter
keeps the default string ``"default prompt"``.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


DEFAULT_PROMPT = "default prompt"


def _ask_language_instruction(default_value: str) -> str:
    """Prompt the operator once for a language instruction.

    Falls back to ``default_value`` when stdin is not a TTY (e.g. launched
    from a script) or when the user just hits Enter. Any leading/trailing
    whitespace is stripped.
    """
    prompt = (
        "Language instruction for this recording session "
        f"[default: {default_value!r}]: "
    )
    if not sys.stdin or not sys.stdin.isatty():
        print(f"[data_recorder] stdin not a TTY; using default prompt {default_value!r}")
        return default_value
    try:
        entered = input(prompt)
    except EOFError:
        entered = ""
    entered = (entered or "").strip()
    return entered or default_value


def _launch_setup(context, *_args, **_kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    sample_hz = float(LaunchConfiguration("sample_hz").perform(context))
    ring_capacity = int(LaunchConfiguration("ring_capacity").perform(context))
    output_directory = LaunchConfiguration("output_directory").perform(context)
    file_prefix = LaunchConfiguration("file_prefix").perform(context)
    enable_cameras = LaunchConfiguration("enable_cameras").perform(context)
    wrist_camera_serial = LaunchConfiguration("wrist_camera_serial").perform(context)
    exterior_camera_serial = LaunchConfiguration("exterior_camera_serial").perform(context)
    placeholder_camera_serial = LaunchConfiguration("placeholder_camera_serial").perform(context)
    camera_width = int(LaunchConfiguration("camera_width").perform(context))
    camera_height = int(LaunchConfiguration("camera_height").perform(context))
    camera_fps = int(LaunchConfiguration("camera_fps").perform(context))
    camera_working_dir_base = LaunchConfiguration("camera_working_dir_base").perform(
        context
    )
    camera_queue_maxsize = int(LaunchConfiguration("camera_queue_maxsize").perform(context))
    annotations_filename = LaunchConfiguration("annotations_filename").perform(context)

    cli_default = LaunchConfiguration("language_instruction").perform(context) or DEFAULT_PROMPT
    language_instruction = _ask_language_instruction(cli_default)
    print(f"[data_recorder] language_instruction='{language_instruction}'")

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

    from ament_index_python.packages import get_package_prefix

    install_base = get_package_prefix("data_recorder")
    candidate_scripts = [
        # ament_python console_scripts are typically installed here.
        os.path.join(install_base, "lib", "data_recorder", "data_recorder"),
        # keep a fallback for layouts that install console scripts to bin.
        os.path.join(install_base, "bin", "data_recorder"),
    ]
    entry_script = next((p for p in candidate_scripts if os.path.exists(p)), "")

    if not entry_script:
        candidates_str = ", ".join(candidate_scripts)
        raise FileNotFoundError(
            "data_recorder executable not found in expected locations: "
            f"{candidates_str}; build the workspace first."
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
        "-p", f"language_instruction:={language_instruction}",
        "-p", f"annotations_filename:={annotations_filename}",
        "-p", f"enable_cameras:={enable_cameras}",
        # Quote numeric-looking serials so ROS2 infers STRING, not INTEGER.
        "-p", f'wrist_camera_serial:="{wrist_camera_serial}"',
        "-p", f'exterior_camera_serial:="{exterior_camera_serial}"',
        "-p", f"placeholder_camera_serial:={placeholder_camera_serial}",
        "-p", f"camera_width:={camera_width}",
        "-p", f"camera_height:={camera_height}",
        "-p", f"camera_fps:={camera_fps}",
        "-p", f"camera_queue_maxsize:={camera_queue_maxsize}",
    ]
    # Empty string is invalid for `-p name:=` on the CLI; omit so the node default applies.
    _cam_base = (camera_working_dir_base or "").strip()
    if _cam_base:
        cmd.extend(["-p", f"camera_working_dir_base:={_cam_base}"])

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
            DeclareLaunchArgument("language_instruction", default_value=DEFAULT_PROMPT),
            DeclareLaunchArgument(
                "annotations_filename",
                default_value="aggregated-annotations-latest.json",
            ),
            DeclareLaunchArgument("enable_cameras", default_value="true"),
            DeclareLaunchArgument("wrist_camera_serial", default_value="352122273343"),
            DeclareLaunchArgument("exterior_camera_serial", default_value="311322303242"),
            DeclareLaunchArgument("placeholder_camera_serial", default_value="zero_exterior_2"),
            DeclareLaunchArgument("camera_width", default_value="640"),
            DeclareLaunchArgument("camera_height", default_value="480"),
            DeclareLaunchArgument("camera_fps", default_value="30"),
            DeclareLaunchArgument(
                "camera_working_dir_base",
                default_value="",
                description="Optional base dir for temp MP4s (e.g. /dev/shm); empty uses system temp.",
            ),
            DeclareLaunchArgument(
                "camera_queue_maxsize",
                default_value="64",
                description="Bounded async frame queue; full queue drops new frames (returns False tick).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
