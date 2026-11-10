"""Gazebo Harmonic（エレベーターの世界）、ros_gz_bridge、エレベーターの制御を起動する.

  ros2 launch elevator_sim sim.launch.py

引数:
  gui                 true で Gazebo の画面を出す（既定）. false で画面なし
  headless_rendering  画面なしのとき、カメラの描画に EGL を使う（既定 true. NVIDIA の GPU 向け）
                      false にすると DISPLAY の X サーバ（Xvfb など）で描画する
  verbosity           gz sim のログの詳しさ（0〜4）
  board_floor / dest_floor / start_floor   乗る階 / 行先階 / かごの初期位置
  door_hold_s         扉を全開で保持する秒数
  door_sensor         出入口に物があれば閉めない・開き直す（既定 true）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _gazebo(context, *args, **kwargs):
    pkg = get_package_share_directory("elevator_sim")
    world = os.path.join(pkg, "worlds", "elevator.sdf")
    cmd = ["gz", "sim", "-r", "-v", LaunchConfiguration("verbosity").perform(context), world]
    if not _truthy(LaunchConfiguration("gui").perform(context)):
        cmd.append("-s")
        if _truthy(LaunchConfiguration("headless_rendering").perform(context)):
            cmd.append("--headless-rendering")
    return [ExecuteProcess(cmd=cmd, output="screen", shell=False)]


def generate_launch_description():
    pkg = get_package_share_directory("elevator_sim")
    models = os.path.join(pkg, "models")
    resource_path = models
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_path += os.pathsep + os.environ["GZ_SIM_RESOURCE_PATH"]

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("headless_rendering", default_value="true"),
        DeclareLaunchArgument("verbosity", default_value="2"),
        DeclareLaunchArgument("board_floor", default_value="1"),
        DeclareLaunchArgument("dest_floor", default_value="3"),
        DeclareLaunchArgument("start_floor", default_value="1"),
        DeclareLaunchArgument("call_delay_s", default_value="3.0"),
        DeclareLaunchArgument("door_hold_s", default_value="6.0"),
        DeclareLaunchArgument("door_sensor", default_value="true"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        OpaqueFunction(function=_gazebo),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            parameters=[{"config_file": os.path.join(pkg, "config", "bridge.yaml"), "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="elevator_sim",
            executable="elevator_controller",
            name="elevator_controller",
            parameters=[{
                "use_sim_time": True,
                "board_floor": typed("board_floor", int),
                "dest_floor": typed("dest_floor", int),
                "start_floor": typed("start_floor", int),
                "call_delay_s": typed("call_delay_s", float),
                "door_hold_s": typed("door_hold_s", float),
                "door_sensor": typed("door_sensor", bool),
            }],
            output="screen",
        ),
    ])
