#!/usr/bin/env python3

# ------------------------------------------------------------------------------
# Developer and Contact:
# Marios Xanthidis
# Research Scientist @ SINTEF Ocean
# Email: marios.xanthidis@sintef.no
#
# License:
# This software is released under a permissive free-use license.
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files, to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the software,
# subject to the following conditions:
# - The above contact and attribution notice shall be included in all copies or
#   substantial portions of the software.
# - This software is provided "as is", without warranty of any kind, expressed
#   or implied.
#
# Aknowledgements:
# - Supported by the Research Council of Norway (EchoNav: NO-359447)
# - Filtering and name conventions adapted from Alberto Quattrini Li @ Dartmouth
#   His repository for ROS1 integration of the Sonar 3D-15 can be found in:
#   https://github.com/quattrinili/Sonar-3D-15-api-example/tree/ros1
# ------------------------------------------------------------------------------

"""Replay a Sonar 3D-15 .sonar recording as ROS 2 messages.

Reads a .sonar file (Range Image Protocol RIP1 or RIP2 packets) using the
`wlsonar` package and publishes its contents as ROS 2 messages, paced to the
original packet timestamps. Record the published topics to a bag with
`ros2 bag record` in a separate terminal.

Usage:
    1. Start recording in one terminal:
        ros2 bag record -o <output_bag_dir> \\
            /sonar3d/point_cloud /sonar3d/range_image /sonar3d/signal_strength_image
    2. In another terminal, run:
        ros2 run sonar3d sonar_to_bag --file <recording.sonar> --realtime-factor 1.0
"""

import argparse
import datetime
import time

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header

import wlsonar.range_image_protocol as rip
from sonar3d.ros_conversions import (
    bitmap_image_to_image_msg,
    range_image_to_pointcloud2,
    range_image_to_range_image_msg,
)

# Reject obviously invalid timestamps (e.g. unsynchronised sonar clock at boot).
YEAR_CHECK = 2023


def make_ros2_time(stamp_seconds: float) -> TimeMsg:
    """Build a builtin_interfaces/Time from a float epoch-seconds value."""
    sec = int(stamp_seconds)
    nanosec = int(round((stamp_seconds - sec) * 1e9))
    t = TimeMsg()
    t.sec = sec
    t.nanosec = nanosec
    return t


class SonarFilePlayer(Node):
    """Plays back Range Image Protocol packets from a file as ROS 2 messages."""

    def __init__(self, frame_id: str, realtime_factor: float):
        super().__init__('sonar3d_file_player')
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, 'sonar3d/point_cloud', 10)
        self.range_image_publisher_ = self.create_publisher(Image, 'sonar3d/range_image', 10)
        self.signal_strength_publisher_ = self.create_publisher(
            Image, 'sonar3d/signal_strength_image', 10
        )
        self.frame_id = frame_id
        self.realtime_factor = realtime_factor

    def _stamp_seconds(self, msg) -> float | None:
        """Return the epoch-seconds timestamp of msg, or None if missing/invalid."""
        if not (msg.HasField('header') and msg.header.HasField('timestamp')):
            return None
        # ToDatetime() returns a naive UTC datetime; mark it UTC for a correct epoch.
        dt = msg.header.timestamp.ToDatetime().replace(tzinfo=datetime.timezone.utc)
        if dt.year < YEAR_CHECK:
            return None
        return dt.timestamp()

    def play(self, path: str) -> None:
        prev_stamp = None
        count = 0
        with open(path, 'rb') as f:
            while rclpy.ok():
                try:
                    msg = rip.unpack(
                        f, known_message_types=(rip.RangeImage, rip.BitmapImageGreyscale8)
                    )
                except rip.UnknownProtobufTypeError:
                    # Undocumented internal message type: skip.
                    continue
                except EOFError:
                    break
                except rip.CRCMismatchError as e:
                    # The packet length was valid, so we can skip past it and continue.
                    self.get_logger().warning(f'CRC mismatch, skipping packet: {e}')
                    continue
                except (rip.BadIDError, ValueError) as e:
                    # Framing is lost; we cannot reliably find the next packet.
                    self.get_logger().warning(f'Stopping playback, cannot parse stream: {e}')
                    break

                stamp = self._stamp_seconds(msg)
                if stamp is None:
                    continue

                # Pace publishing to the original inter-packet timing.
                if prev_stamp is not None:
                    dt = (stamp - prev_stamp) / self.realtime_factor
                    if dt > 0:
                        time.sleep(dt)
                prev_stamp = stamp

                header = Header()
                header.stamp = make_ros2_time(stamp)
                header.frame_id = self.frame_id

                if isinstance(msg, rip.RangeImage):
                    cloud = range_image_to_pointcloud2(msg, header)
                    range_image = range_image_to_range_image_msg(msg, header)
                    self.pointcloud_publisher_.publish(cloud)
                    self.range_image_publisher_.publish(range_image)
                    self.get_logger().info(f'Published range image + point cloud at {stamp:.3f}')
                elif isinstance(msg, rip.BitmapImageGreyscale8):
                    self.signal_strength_publisher_.publish(bitmap_image_to_image_msg(msg, header))
                    self.get_logger().info(f'Published signal strength image at {stamp:.3f}')

                count += 1
                # Let publishers flush without blocking.
                rclpy.spin_once(self, timeout_sec=0)

        self.get_logger().info(f'Done publishing {count} messages.')


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description='Replay a Sonar 3D-15 .sonar recording as ROS 2 messages.'
    )
    parser.add_argument('--file', type=str, required=True, help='.sonar recording to replay.')
    parser.add_argument(
        '--realtime-factor',
        type=float,
        default=1.0,
        help='Playback speed multiplier (1.0=real time, 2.0=2x, 0.5=half).',
    )
    parser.add_argument(
        '--frame-id', type=str, default='sonar3d', help='frame_id for published messages.'
    )
    # parse_known_args so ROS arguments (e.g. --ros-args) are ignored gracefully.
    cli, _ = parser.parse_known_args()

    node = SonarFilePlayer(frame_id=cli.frame_id, realtime_factor=cli.realtime_factor)
    try:
        node.play(cli.file)
    except FileNotFoundError:
        node.get_logger().error(f'File not found: {cli.file}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
