from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    offboard_control_node = Node(
        package='px4_ros_com',
        executable='offboard_control',
        name='offboard_control',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        offboard_control_node
    ])
