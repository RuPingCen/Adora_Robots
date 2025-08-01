from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adora_lifting_motor_control',
            executable='adora_lifting_motor_control_node',
            name='adora_lifting_motor_control_node',
            output='screen',
            parameters=[
                {'dev': '/dev/ttyACM0'},
                {'baud': 19200},
                {'max_lifting_distance': 700},#不可修改
                {'robot_name': 'ADORA_A2_MAX'},#不可修改
                {'sub_cmdvel_topic': '/adora/lifting_motor/cmd'},
                {'pub_position_topic': '/adora/lifting_motor/states'}
            ]
        )
    ])

 
