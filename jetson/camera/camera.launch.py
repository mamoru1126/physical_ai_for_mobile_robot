"""USB カメラ（usb_cam）と、ブラウザ向けの配信（web_video_server）を起動する.

カメラのノードが止まったら（USB が抜けたなど）、全体を終わらせる. コンテナごと再起動され、
カメラがつながるのを待ってから起動し直す.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    "video_device": ("/dev/video0", "カメラのデバイス"),
    "image_width": ("640", "横の画素数"),
    "image_height": ("480", "縦の画素数"),
    "framerate": ("30.0", "1 秒あたりの枚数"),
    "pixel_format": ("mjpeg2rgb", "カメラから受け取る形式（mjpeg2rgb、yuyv2rgb など）"),
    "namespace": ("usb_cam", "トピックの名前空間（<namespace>/image_raw になる）"),
    "frame_id": ("usb_cam", "画像の座標系の名前"),
    "camera_info_url": ("", "カメラの校正ファイル（file:///... . 空なら無し）"),
    "port": ("9000", "ブラウザで見るためのポート"),
}


def generate_launch_description() -> LaunchDescription:
    actions = [DeclareLaunchArgument(name, default_value=default, description=desc)
               for name, (default, desc) in ARGS.items()]
    arg = {name: LaunchConfiguration(name) for name in ARGS}
    # 型を決めて渡す（「30」と書かれても小数、空の文字列も文字列として渡すため）
    as_int = {n: ParameterValue(arg[n], value_type=int) for n in ("image_width", "image_height", "port")}
    as_str = {n: ParameterValue(arg[n], value_type=str)
              for n in ("video_device", "pixel_format", "frame_id", "camera_info_url")}
    camera = Node(
        package="usb_cam", executable="usb_cam_node_exe", name="usb_cam", namespace=arg["namespace"],
        output="screen",
        parameters=[{
            "video_device": as_str["video_device"],
            "image_width": as_int["image_width"],
            "image_height": as_int["image_height"],
            "framerate": ParameterValue(arg["framerate"], value_type=float),
            "pixel_format": as_str["pixel_format"],
            "camera_name": "usb_cam",
            "frame_id": as_str["frame_id"],
            "camera_info_url": as_str["camera_info_url"],
            "io_method": "mmap",
        }],
        on_exit=Shutdown(),
    )
    web = Node(
        package="web_video_server", executable="web_video_server", name="web_video_server", output="screen",
        parameters=[{"port": as_int["port"], "address": "0.0.0.0"}],
        on_exit=Shutdown(),
    )
    return LaunchDescription(actions + [camera, web])
