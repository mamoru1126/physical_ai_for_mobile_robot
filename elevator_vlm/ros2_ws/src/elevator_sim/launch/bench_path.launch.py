"""道のベンチ: 実写の映像（ATC のコースのフレーム）を流すダミーの世界と実行系をまとめて起動する. Gazebo は使わない.

  ros2 launch elevator_sim bench_path.launch.py              # 本物の VLM サーバ（環境変数 VLM_BASE_URL）を使う
  ros2 launch elevator_sim bench_path.launch.py vlm:=mock    # 正解（labels.json）から答えるモック VLM（CI 用）

つながり（bench.launch.py と同じ）:
  task_node → cmd_vel_raw → safety_gate → <cmd_vel_topic>
  external_gate:=true（既定）のとき、<cmd_vel_topic> は /vlm_cmd_vel. 点群を見る安全ゲート（別のコンテナ）が
  /cmd_vel にして、道のベンチに渡す. false のときは、直接 /cmd_vel に出す.

引数:
  vlm               real（既定）か mock
  external_gate     点群を見る安全ゲートを後ろにつなぐか（既定 true）
  web_port          Web UI のポート（既定 8080）
  log_dir           記録の保存先（既定 /runs）
  frames_dir        映像のフレーム（既定 /atc_frames）
  first / last      使うフレームの番号の範囲（既定 1〜最後）
  meters_per_frame  前に何 m 走ったら 1 枚進めるか（既定 0.6）
  labels            モック VLM が使う正解（既定 <frames_dir>/labels.json）
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
        labels = LaunchConfiguration("labels").perform(context) or os.path.join(
            LaunchConfiguration("frames_dir").perform(context), "labels.json")
        actions += [
            SetEnvironmentVariable("VLM_BASE_URL", f"http://127.0.0.1:{MOCK_PORT}/v1"),
            SetEnvironmentVariable("VLM_MODEL", "mock-vlm"),
            Node(package="elevator_sim", executable="mock_vlm", name="mock_vlm",
                 parameters=[{"port": MOCK_PORT, "latency_s": 0.5, "mode": "path", "labels": labels}],
                 output="screen"),
        ]
    external = _truthy(LaunchConfiguration("external_gate").perform(context))
    sim_pkg = get_package_share_directory("elevator_sim")
    task_pkg = get_package_share_directory("elevator_task")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(task_pkg, "launch", "task.launch.py")),
        launch_arguments={
            "config_file": os.path.join(sim_pkg, "config", "task_bench_path.yaml"),
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
        DeclareLaunchArgument("frames_dir", default_value=os.environ.get("ATC_FRAMES_DIR", "/atc_frames")),
        DeclareLaunchArgument("first", default_value="1"),
        DeclareLaunchArgument("last", default_value="0"),
        DeclareLaunchArgument("meters_per_frame", default_value="0.6"),
        DeclareLaunchArgument("labels", default_value=""),
        Node(
            package="elevator_sim",
            executable="bench_path",
            name="bench_path",
            parameters=[{
                "frames_dir": ParameterValue(LaunchConfiguration("frames_dir"), value_type=str),
                "first": ParameterValue(LaunchConfiguration("first"), value_type=int),
                "last": ParameterValue(LaunchConfiguration("last"), value_type=int),
                "meters_per_frame": ParameterValue(LaunchConfiguration("meters_per_frame"), value_type=float),
            }],
            output="screen",
        ),
        OpaqueFunction(function=_setup),
    ])
