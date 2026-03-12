import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_srvs.srv import SetBool
from std_msgs.msg import Header

from sonar3d.api.inspect_sonar_data import parse_rip2_packet, decode_protobuf_packet, rangeImageToXYZ
from sonar3d.api.interface_sonar_api import set_speed, set_mode, get_mode, set_acoustics, get_acoustics, describe_response, enable_multicast
import socket
import struct
import numpy as np

class TimerNode(Node):

    # Multicast group and port used by the Sonar 3D-15
    MULTICAST_GROUP = '224.0.0.96'
    PORT = 4747

    # The maximum possible packet size for Sonar 3D-15 data
    BUFFER_SIZE = 65535

    def __init__(self):
        super().__init__('timer_node')
        
        # Declare parameters
        self.declare_parameter('IP', '192.168.194.96')# '192.168.194.96' is the fallback ip, to change this, edit the launchfile.
        self.declare_parameter('host_IP', '192.168.2.15') #jetson IP
        self.declare_parameter('speed_of_sound', 1491)    # setting this takes ~20s
        self.declare_parameter('max_dist', 5)
        self.declare_parameter('min_dist', 0)

        self.sonar_ip = self.get_parameter('IP').get_parameter_value().string_value
        self.host_ip = self.get_parameter('host_IP').get_parameter_value().string_value
        self.sonar_speed_of_sound = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        self.max_dist = self.get_parameter('max_dist').get_parameter_value().double_value
        self.min_dist = self.get_parameter('min_dist').get_parameter_value().double_value

        self.get_logger().info(f"Sonar data range {self.min_dist} m x {self.max_dist} m")
        # Set speed of sound in sonar API
        resp = set_speed(self.sonar_ip, self.sonar_speed_of_sound)
        self.get_logger().info(f"Set speed of sound: {describe_respose(self.sonar_ip, resp)}")

        # Create a timer that calls the timer_callback every sample_time seconds 
        sample_time = 0.01          # sample time in seconds
        self.create_timer(sample_time, self.timer_callback)
        self.get_logger().info(f'Timer Node initialized with {1/sample_time} Hz')

        # Create a publisher that publishes the point cloud data
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, 'sonar_point_cloud', 10)
        self.image_publisher_ = self.create_publisher(Image, 'sonar_range_image', 10)
        self.acoustic_service_ = self.create_service(SetBool, 'set_acoustics', self.set_acoustics_cb)
        self.acoustic_mode_service_ = self.create_service(SetBool, 'set_acoustic_mode', self.set_acoustic_mode_cb)
        # Enable the acoustics on the sonar
        resp = set_acoustics(self.sonar_ip, True)
        self.get_logger().info(f'Enabling acoustics response: {describe_response(self.sonar_ip, resp)}')

        resp = enable_multicast(self.sonar_ip)
        self.get_logger().info(f'Enabling multicast response: {describe_response(self.sonar_ip, resp)}')

        # Set up a UDP socket with multicast membership
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.PORT))

        group = socket.inet_aton(self.MULTICAST_GROUP)
        mreq = struct.pack('4s4s', group, socket.inet_aton(self.host_ip)) #issue 4 Waterlinked/Sonar-3D-15-ROS-driver
        # mreq = struct.pack('4sl', group, socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.get_logger().info("Joined multicast 224.0.0.96 on enp0s31f6 ({})".format(self.host_ip))

        self.get_logger().info(f"Listening for Sonar 3D-15 RIP1 packets on {self.MULTICAST_GROUP}:{self.PORT}...")

        if self.sonar_ip != "":
            self.get_logger().info(f"Filtering packets from IP: {self.sonar_ip}")


    def timer_callback(self):

        #Receive updates of distance parameters
        self.max_dist = self.get_parameter('max_dist').get_parameter_value().double_value
        self.min_dist = self.get_parameter('min_dist').get_parameter_value().double_value
        sonar_speed_of_sound = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        if self.sonar_speed_of_soud != sonar_speed_of_soud:
            self.get_logger().info(f'Update sonar speed of sound {sonar_speed_of_sound}')
            self.sonar_speed_of_soud = sonar_speed_of_soud
            res = set_speed(self.sonar_ip, self.sonar_speed_of_soud)
            self.get_logger().info(f'{describe_response(self.sonar_ip, resp)}')
        try:
            data, addr = self.sock.recvfrom(self.BUFFER_SIZE)
        except BlockingIOError:
            return
        # If SONAR_IP is configured, and this doesn't match the known Sonar IP, skip it.
        if not (addr[0] == self.sonar_ip or addr[0] == '192.168.194.96'):
            self.get_logger().info(f"Received packet from {addr[0]}. Data was received from an IP that does not match the declared SONAR_IP ({self.sonar_ip}), so the packet will be skipped.")
            return

        payload = parse_rip2_packet(data)
        if payload is None:
            self.get_logger().warning("Parsed payload is None, skipping packet.")
            return
        # Decode the Protobuf message
        result = decode_protobuf_packet(payload)
        if not result:
            self.get_logger().warning("Decoding Protobuf packet failed, skipping packet.")
            return

        msg_type, msg_obj = result

        if msg_type == 'RangeImage':
            # Convert the RangeImage message to voxel data
            voxels = rangeImageToXYZ(msg_obj)

            # extract the x, y, z coordinates from the voxels
            # Filter points based on Euclidean distance criteria
            pts = []
            for i in range(len(voxels)):
                x = voxels[i]['x']
                y = voxels[i]['y']
                z = voxels[i]['z']
                # Calculate Euclidean distance from origin
                distance = np.sqrt(x**2 + y**2 + z**2)
                # Log distance for debugging
                self.get_logger().info(f'Point {i}: x={x:.3f}, y={y:.3f}, z={z:.3f}, dist={distance:.3f}, min={self.min_dist}, max={self.max_dist}')
                # Only include points within the min_dist and max_dist range
                if self.min_dist <= distance <= self.max_dist:
                    pts.append((x, y, z))
                    self.get_logger().info(f'Included point {i}')
                else:
                    self.get_logger().info(f'Filtered out point {i}')

            # Create a PointCloud2 message
            # Create the msg heinspeader
            header = Header()
            header.stamp = self.get_clock().now().to_msg()  # Use ROS2 time
            header.frame_id = 'sonar_frame'

            cloud_msg = point_cloud2.create_cloud_xyz32(header, pts)

            # Publish the PointCloud2 message
            self.pointcloud_publisher_.publish(cloud_msg)

            # Publish the raw range image
            img_msg = Image()
            img_msg.header = header
            img_msg.height = msg_obj.height
            img_msg.width = msg_obj.width
            img_msg.encoding = '32FC1'
            img_msg.is_bigendian = False
            img_msg.step = msg_obj.width * 4
            range_image = (np.array(msg_obj.image_pixel_data, dtype=np.uint32) * msg_obj.image_pixel_scale).astype(np.float32)
            img_msg.data = range_image.tobytes()
            self.image_publisher_.publish(img_msg)

    def set_acoustics_cb(self, request, response):
        if request.data:
            self.get_logger().info("Received request to ENABLE acoustics")
        else:            
            self.get_logger().info("Received request to DISABLE acoustics")
        # call the HTTP helper and keep the ROS response object separate
        _ = set_acoustics(self.sonar_ip, request.data)
        http_resp = get_acoustics(self.sonar_ip)
        if request.data and http_resp:
            response.success = True
        elif not request.data and not http_resp:
            response.success = True
        self.get_logger().info("output {}".format(http_resp))
        response.message = "output {}".format(http_resp)
        return response        
        
    def set_acoustic_mode_cb(self, request, response):
        if request.data:
            self.get_logger().info("Received request to HIGH frequency")
            req = "high-frequency"
        else:            
            self.get_logger().info("Received request to HIGH frequency")
            req = "low-frequency"        # call the HTTP helper and keep the ROS response object separate
        _ = set_mode(self.sonar_ip, req)
        http_resp = get_mode(self.sonar_ip)
        # self.get_logger().info("output {}".format(http_resp))
        response.success = True
        response.message = "{}".format(http_resp)
        return response        
        
    

def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()