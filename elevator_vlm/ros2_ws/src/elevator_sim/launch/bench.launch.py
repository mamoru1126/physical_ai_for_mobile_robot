"""ベンチ: ダミーの世界（実機の代わり）と実行系をまとめて起動する. Gazebo は使わない.

  ros2 launch elevator_sim bench.launch.py              # 本物の VLM サーバ（環境変数 VLM_BASE_URL）を使う
  ros2 launch elevator_sim bench.launch.py vlm:=mock    # 真値から答えるモック VLM を使う（CI 用）

つながり:
  task_node → cmd_vel_raw → safety_gate（速度の上限、途絶の監視）→ <cmd_vel_topic>
  external_gate:=true（既定）のとき、<cmd_vel_topic> は /vlm_cmd_vel. 点群を見る安全ゲート（safety_gate/、
  別のコンテナ）が /cmd_vel にして、ダミーの世界に渡す. false のときは、直接 /cmd_vel に出す.

引数:
  vlm            real（既定）か mock
  external_gate  点群を見る安全ゲートを後ろにつなぐか（既定 true）
  web_port       Web UI のポート（既定 8080）
  log_dir        記録の保存先（既定 /runs）
  board_floor / dest_floor / door_hold_s / door_sensor / call_delay_s   ダミーのエレベーターの設定
  samples_dir    ダミーのカメラの画像（既定 /workspace/samples）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MOCK_PORT = 8001


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _setup(context, *args, **kwargs):
    actions = []
    mode = LaunchConfiguration("vlm").perform(context).strip().lower()
    if mode not in ("real", "mock"):
        raise RuntimeError(f"vlm は real か mock で指定してください（{mode}）")
    if mode == "mock":
        actions += [
            SetEnvironmentVariable("VLM_BASE_URL", f"http://127.0.0.1:{MOCK_PORT}/v1"),
            SetEnvironmentVariable("VLM_MODEL", "mock-vlm"),
            Node(package="elevator_sim", executable="mock_vlm", name="mock_vlm",
                 parameters=[{"port": MOCK_PORT, "latency_s": 0.3}], output="screen"),
        ]
    external = _truthy(LaunchConfiguration("external_gate").perform(context))
    sim_pkg = get_package_share_directory("elevator_sim")
    task_pkg = get_package_share_directory("elevator_task")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(task_pkg, "launch", "task.launch.py")),
        launch_arguments={
            "config_file": os.path.join(sim_pkg, "config", "task_bench.yaml"),
            "web_port": LaunchConfiguration("web_port"),
            "log_dir": LaunchConfiguration("log_dir"),
            "use_sim_time": "false",
            "cmd_vel_topic": "/vlm_cmd_vel" if external else "/cmd_vel",
        }.items(),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vlm", default_value="real"),
        DeclareLaunchArgument("external_gate", default_value="true"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("log_dir", default_value="/runs"),
        DeclareLaunchArgument("board_floor", default_value="1"),
        DeclareLaunchArgument("dest_floor", default_value="3"),
        DeclareLaunchArgument("door_hold_s", default_value="15.0"),
        DeclareLaunchArgument("door_sensor", default_value="true"),
        DeclareLaunchArgument("call_delay_s", default_value="3.0"),
        DeclareLaunchArgument("samples_dir", default_value=os.environ.get("ELEVATOR_SAMPLES_DIR", "/workspace/samples")),
        Node(
            package="elevator_sim",
            executable="bench_world",
            name="bench_world",
            parameters=[{
                "board_floor": ParameterValue(LaunchConfiguration("board_floor"), value_type=int),
                "dest_floor": ParameterValue(LaunchConfiguration("dest_floor"), value_type=int),
                "door_hold_s": ParameterValue(LaunchConfiguration("door_hold_s"), value_type=float),
                "door_sensor": ParameterValue(LaunchConfiguration("door_sensor"), value_type=bool),
                "call_delay_s": ParameterValue(LaunchConfiguration("call_delay_s"), value_type=float),
                "samples_dir": ParameterValue(LaunchConfiguration("samples_dir"), value_type=str),
            }],
            output="screen",
        ),
        OpaqueFunction(function=_setup),
    ])
