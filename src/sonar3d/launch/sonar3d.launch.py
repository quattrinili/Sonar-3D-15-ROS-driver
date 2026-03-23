from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sonar3d',
            executable='sonar_publisher',
            name='sonar_node',
            output='screen',
            parameters=[
                {'IP': '192.168.194.96'},  # Change to your sonar IP, '192.168.194.96' is the fallback ip.
                {'speed_of_sound': 1491},
                {'realtime': False} # Set to False to read from bag file instead of live sonar
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/cmb/singularity/2_ws/3dsonar_ws/src/Sonar-3D-15-ROS-driver/src/sonar3d/launch/view_raw_pc.rviz']
        )
    ])
