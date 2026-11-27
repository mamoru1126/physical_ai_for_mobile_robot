"""実行系（task_node と safety_gate）を起動する.

  ros2 launch elevator_task task.launch.py

引数:
  config_file   設定ファイル（既定はこのパッケージの config/task.yaml）
  web_port      Web UI のポート（既定 8080）
  log_dir       記録の保存先（既定 /runs）
  use_sim_time  シミュレーション時刻を使うか（既定 true. 実機では false）
  cmd_vel_topic 安全ゲート（このパッケージの safety_gate）が速度指令を出すトピック（既定 cmd_vel）.
                後ろにもう1つ安全ゲート（点群を見るもの）をつなぐときは、その入力（例 /vlm_cmd_vel）にする
  odom_topic    オドメトリ（nav_msgs/Odometry）を受けるトピック（既定 odom. 実機ではロボットの名前に合わせる）
VLM サーバの場所は、環境変数 VLM_BASE_URL / VLM_MODEL か設定ファイルで指定する.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("elevator_task")
    config_file = LaunchConfiguration("config_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=os.path.join(pkg, "config", "task.yaml")),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("log_dir", default_value="/runs"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),
        DeclareLaunchArgument("odom_topic", default_value="odom"),
        Node(
            package="elevator_task",
            executable="safety_gate",
            name="safety_gate",
            remappings=[("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
                        ("odom", LaunchConfiguration("odom_topic"))],
            parameters=[{
                "config_file": ParameterValue(config_file, value_type=str),
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        ),
        Node(
            package="elevator_task",
            executable="task_node",
            name="elevator_task",
            remappings=[("odom", LaunchConfiguration("odom_topic"))],
            parameters=[{
                "config_file": ParameterValue(config_file, value_type=str),
                "web_port": ParameterValue(LaunchConfiguration("web_port"), value_type=int),
                "log_dir": ParameterValue(LaunchConfiguration("log_dir"), value_type=str),
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        ),
    ])
