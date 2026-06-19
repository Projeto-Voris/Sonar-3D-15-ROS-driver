"""Convert Water Linked Range Image Protocol messages to ROS 2 messages.

These helpers wrap the conversion utilities from the `wlsonar` package
(https://pypi.org/project/wlsonar/) so that both the live driver
(`multicast_listener.py`) and the file player (`sonar_to_bag.py`) share a single,
tested conversion path.

Coordinate convention follows `wlsonar.range_image_to_xyz`:
    x: forward (range direction)
    y: left/right (horizontal field of view)
    z: up/down (vertical field of view)
"""

import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import wlsonar
import wlsonar.range_image_protocol as rip


def range_image_to_pointcloud2(range_image: rip.RangeImage, header: Header) -> PointCloud2:
    """Convert a RangeImage to a PointCloud2 of xyz points (meters).

    Pixels without a return are dropped, so the resulting cloud is unordered
    (height == 1) and dense.
    """
    voxels = wlsonar.range_image_to_xyz(range_image)
    points = [voxel for voxel in voxels if voxel is not None]
    return point_cloud2.create_cloud_xyz32(header, points)


def range_image_to_range_image_msg(range_image: rip.RangeImage, header: Header) -> Image:
    """Convert a RangeImage to a 32FC1 ROS Image of distances in meters.

    Each pixel holds the distance (in meters) to the strongest reflection in
    that direction, or 0.0 where there is no return.
    """
    distances = np.array(wlsonar.range_image_to_distance(range_image), dtype=np.float32)

    msg = Image()
    msg.header = header
    msg.height = range_image.height
    msg.width = range_image.width
    msg.encoding = "32FC1"
    msg.is_bigendian = False
    msg.step = range_image.width * 4  # 4 bytes per float32 pixel
    msg.data = distances.tobytes()
    return msg


def bitmap_image_to_image_msg(bitmap_image: rip.BitmapImageGreyscale8, header: Header) -> Image:
    """Convert a BitmapImageGreyscale8 to a mono8 ROS Image.

    The image is flipped vertically so that it is oriented the same way as the
    Water Linked replayer and the wlsonar conversion examples.
    """
    pixels = np.frombuffer(bitmap_image.image_pixel_data, dtype=np.uint8)
    pixels = pixels.reshape(bitmap_image.height, bitmap_image.width)
    pixels = np.flipud(pixels)

    msg = Image()
    msg.header = header
    msg.height = bitmap_image.height
    msg.width = bitmap_image.width
    msg.encoding = "mono8"
    msg.is_bigendian = False
    msg.step = bitmap_image.width  # 1 byte per pixel
    msg.data = pixels.tobytes()
    return msg
