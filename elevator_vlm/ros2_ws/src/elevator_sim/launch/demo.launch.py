"""シミュレーションと実行系をまとめて起動する.

  ros2 launch elevator_sim demo.launch.py              # 本物の VLM サーバを使う
  ros2 launch elevator_sim demo.launch.py vlm:=mock    # 真値から答えるモック VLM を使う

引数（sim.launch.py の引数に加えて）:
  vlm           real（既定. 環境変数 VLM_BASE_URL / VLM_MODEL の VLM サーバを使う）か mock
  web_port      Web UI のポート（既定 8080）
  log_dir       記録の保存先（既定 /runs）
  mock_latency_s / mock_point_noise_px / mock_check_flip_prob   モック VLM の遅れ、指差しのずれ、誤答の確率
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SIM_ARGS = ("gui", "headless_rendering", "verbosity", "board_floor", "dest_floor", "start_floor",
            "call_delay_s", "door_hold_s", "door_sensor")
MOCK_PORT = 8001


def _vlm(context, *args, **kwargs):
    mode = LaunchConfiguration("vlm").perform(context).strip().lower()
    if mode not in ("real", "mock"):
        raise RuntimeError(f"vlm は real か mock で指定してください（{mode}）")
    if mode == "real":
        return []
    return [
        SetEnvironmentVariable("VLM_BASE_URL", f"http://127.0.0.1:{MOCK_PORT}/v1"),
        SetEnvironmentVariable("VLM_MODEL", "mock-vlm"),
        Node(
            package="elevator_sim",
            executable="mock_vlm",
            name="mock_vlm",
            parameters=[{
                "use_sim_time": True,
                "port": MOCK_PORT,
                "latency_s": ParameterValue(LaunchConfiguration("mock_latency_s"), value_type=float),
                "point_noise_px": ParameterValue(LaunchConfiguration("mock_point_noise_px"), value_type=float),
                "check_flip_prob": ParameterValue(LaunchConfiguration("mock_check_flip_prob"), value_type=float),
            }],
            output="screen",
        ),
    ]


def generate_launch_description():
    sim_pkg = get_package_share_directory("elevator_sim")
    task_pkg = get_package_share_directory("elevator_task")
    sim_defaults = {"gui": "true", "headless_rendering": "true", "verbosity": "2", "board_floor": "1",
                    "dest_floor": "3", "start_floor": "1", "call_delay_s": "3.0", "door_hold_s": "6.0",
                    "door_sensor": "true"}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=sim_defaults[name]) for name in SIM_ARGS] + [
            DeclareLaunchArgument("vlm", default_value="real"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("log_dir", default_value="/runs"),
            DeclareLaunchArgument("mock_latency_s", default_value="0.3"),
            DeclareLaunchArgument("mock_point_noise_px", default_value="0.0"),
            DeclareLaunchArgument("mock_check_flip_prob", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "sim.launch.py")),
                launch_arguments={name: LaunchConfiguration(name) for name in SIM_ARGS}.items(),
            ),
            # モックのときは、ここで VLM サーバの場所を差し替えてから実行系を起動する
            OpaqueFunction(function=_vlm),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(task_pkg, "launch", "task.launch.py")),
                launch_arguments={
                    "config_file": os.path.join(sim_pkg, "config", "task_sim.yaml"),
                    "web_port": LaunchConfiguration("web_port"),
                    "log_dir": LaunchConfiguration("log_dir"),
                    "use_sim_time": "true",
                }.items(),
            ),
        ])
