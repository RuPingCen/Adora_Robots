from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adora_chassis_bringup',
            executable='adora_chassis_bringup_node',
            name='adora_chassis_bringup_node',
            namespace='adora_robot/chassis',
            output='screen',
            parameters=[
                {'usart_port': '/dev/ttyACM0'},
                {'baud_data': 115200},
                {'control_mode': 1},
                ]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint2base_link',
        #     arguments=['0.0', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_footprint','base_link']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link2laser_link',
        #     arguments=['0.07', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link2imu',
        #     arguments=['0.1653', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_node']
        # ),
    ])

 
