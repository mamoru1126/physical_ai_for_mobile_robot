"""安全ゲートと、その WebUI を起動する. どちらかが止まったら、全部を止める（コンテナごと起動し直す）.

  ros2 launch safety_gate safety_gate.launch.py config:=/config/config.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="/config/config.yaml",
                              description="設定ファイル（コンテナの外から差し込む）"),
        Node(package="safety_gate", executable="safety_gate_node", name="safety_gate_node",
             output="screen", parameters=[{"config_file": config}], on_exit=Shutdown()),
        Node(package="safety_gate", executable="safety_gate_web.py", name="safety_gate_web",
             output="screen", arguments=["--config", config], on_exit=Shutdown()),
    ])
