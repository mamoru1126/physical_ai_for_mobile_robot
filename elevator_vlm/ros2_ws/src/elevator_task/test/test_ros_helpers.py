"""ROS 2 のノードの小さな関数の試験（ROS 2 が入っている環境だけで動く. Docker のイメージを作るときに通す）."""
import io

import pytest

pytest.importorskip("rclpy")

from sensor_msgs.msg import CompressedImage  # noqa: E402

from elevator_task.task_node import compressed_to_rgb8, is_compressed_topic  # noqa: E402


def test_compressed_topic_names():
    assert is_compressed_topic("/camera/compressed")
    assert is_compressed_topic("camera/image_raw/compressed")
    assert not is_compressed_topic("camera/image_raw")


def test_compressed_to_rgb8():
    from PIL import Image

    img = Image.new("RGB", (8, 6), (10, 20, 30))
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    msg = CompressedImage()
    msg.format = "png"
    msg.data = buf.getvalue()
    width, height, rgb = compressed_to_rgb8(msg)
    assert (width, height) == (8, 6)
    assert rgb[:3] == bytes([10, 20, 30])

    broken = CompressedImage()
    broken.data = b"not an image"
    assert compressed_to_rgb8(broken) is None
