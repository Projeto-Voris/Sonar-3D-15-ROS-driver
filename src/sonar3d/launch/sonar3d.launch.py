from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sonar3d',
            executable='sonar_publisher',
            name='sonar3d_node',
            output='screen',
            parameters=[
                # Change to your sonar's IP. '192.168.194.96' is the fallback IP.
                {'IP': '192.168.194.96'},
                # Speed of sound in m/s. 0.0 leaves the sonar's current setting
                # unchanged. Setting a value can take ~20 s to apply.
                {'speed_of_sound': 0.0},
                # frame_id used for published PointCloud2 / Image headers.
                {'frame_id': 'sonar3d'},
            ]
        )
    ])
