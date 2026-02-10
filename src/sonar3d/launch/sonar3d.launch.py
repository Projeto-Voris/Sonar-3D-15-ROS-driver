from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for the sonar3d node'),
        DeclareLaunchArgument('ip', default_value='192.168.194.96', description='IP address of the sonar device'),
        DeclareLaunchArgument('speed_of_sound', default_value='1491', description='Speed of sound in water in m/s'),

        Node(
            package='sonar3d',
            executable='sonar_publisher',
            name='sonar_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'IP': LaunchConfiguration('ip')},
                {'speed_of_sound': LaunchConfiguration('speed_of_sound')}
            ]
        )
    ])
