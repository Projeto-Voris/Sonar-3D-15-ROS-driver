"""ROS 2 driver node for the Water Linked Sonar 3D-15.

Configures the sonar over its HTTP API, listens for Range Image Protocol packets
(RIP1 and RIP2) over UDP multicast using the `wlsonar` package, and republishes
the data as standard ROS 2 messages.
"""

import rclpy
import requests
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header

import wlsonar
import wlsonar.range_image_protocol as rip
from sonar3d.ros_conversions import (
    bitmap_image_to_image_msg,
    range_image_to_pointcloud2,
    range_image_to_range_image_msg,
)


class Sonar3DNode(Node):
    """Driver node that configures the Sonar 3D-15 and republishes its data."""

    def __init__(self):
        super().__init__('sonar3d_node')

        # Parameters
        self.declare_parameter('IP', wlsonar.FALLBACK_IP)
        # speed_of_sound in m/s. 0.0 means "leave the sonar's current setting unchanged".
        # Note: changing speed of sound on the sonar can take ~20 seconds.
        self.declare_parameter('speed_of_sound', 0.0)
        self.declare_parameter('frame_id', 'sonar3d')

        self.sonar_ip = self.get_parameter('IP').get_parameter_value().string_value
        speed_of_sound = self.get_parameter('speed_of_sound').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Source IPs we have already warned about, to avoid log spam.
        self._warned_ips = set()

        # Publishers
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, 'sonar3d/point_cloud', 10)
        self.range_image_publisher_ = self.create_publisher(Image, 'sonar3d/range_image', 10)
        self.signal_strength_publisher_ = self.create_publisher(
            Image, 'sonar3d/signal_strength_image', 10
        )

        # Configure the sonar over its HTTP API.
        try:
            sonar = wlsonar.Sonar3D(self.sonar_ip)
            self.get_logger().info(
                f'Connected to Sonar 3D-15 at {self.sonar_ip} '
                f'(release {sonar.sonar_version})'
            )

            sonar.set_acoustics_enabled(True)
            self.get_logger().info('Enabled acoustics')

            if speed_of_sound > 0.0:
                self.get_logger().info(
                    f'Setting speed of sound to {speed_of_sound} m/s (this can take ~20 s)...'
                )
                sonar.set_speed_of_sound(speed_of_sound)

            sonar.set_udp_multicast()
            self.get_logger().info('Configured sonar for UDP multicast output')
        except (requests.RequestException, RuntimeError, wlsonar.VersionException) as e:
            self.get_logger().error(f'Failed to configure Sonar 3D-15 at {self.sonar_ip}: {e}')
            raise

        # Open the multicast socket. A timeout keeps the node responsive to shutdown.
        self.sock = wlsonar.open_sonar_udp_multicast_socket()
        self.sock.settimeout(1.0)
        self.get_logger().info(
            f'Listening for Sonar 3D-15 RIP1/RIP2 packets on '
            f'{wlsonar.DEFAULT_MCAST_GRP}:{wlsonar.DEFAULT_MCAST_PORT}...'
        )
        self.get_logger().info(f'Filtering packets from sonar IP: {self.sonar_ip}')

        # Drive reception from a timer. recvfrom blocks (up to the socket timeout),
        # so the effective rate is governed by packet arrival.
        self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(wlsonar.UDP_MAX_DATAGRAM_SIZE)
        except TimeoutError:
            return

        # Only accept packets from the configured sonar IP (warn once per source IP).
        if addr[0] != self.sonar_ip:
            if addr[0] not in self._warned_ips:
                self._warned_ips.add(addr[0])
                self.get_logger().warning(
                    f'Ignoring packets from {addr[0]} '
                    f'(does not match configured IP {self.sonar_ip})'
                )
            return

        try:
            msg = rip.unpackb(data)
        except rip.UnknownProtobufTypeError:
            # Undocumented internal message type: silently skip.
            return
        except (rip.BadIDError, rip.CRCMismatchError, rip.ExtraDataError, ValueError) as e:
            self.get_logger().warning(f'Skipping malformed packet: {e}')
            return

        header = self._make_header()

        if isinstance(msg, rip.RangeImage):
            self.pointcloud_publisher_.publish(range_image_to_pointcloud2(msg, header))
            self.range_image_publisher_.publish(range_image_to_range_image_msg(msg, header))
        elif isinstance(msg, rip.BitmapImageGreyscale8):
            self.signal_strength_publisher_.publish(bitmap_image_to_image_msg(msg, header))

    def _make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Sonar3DNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.sock.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
