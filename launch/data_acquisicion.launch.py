from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_acquisicion_charlie',
            executable='pose_broadcaster_node',
            name='pose_broadcaster_node'
        ),
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'bag', 'record',
        #         '/cmd_vel_ctrl',
        #         '/robot1/pose',
        #         '/imu/data_raw',
        #         '/imu/mag'
        #     ],
        #     output='screen'
        # )
    ])
