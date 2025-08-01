from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adora_chassis_bringup',
            executable='adora_a2_max_node',
            name='adora_a2_max_node',
            output='screen',
            parameters=[{
                'dt_port': '/dev/ttyUSB0',
                'dt_baudrate': 115200,
                'dt_odom_enable': True,
                'dt_drive_type': 'SDFZ', # 'SDFZ' of 'HLS'
                'dt_log_display': True,
                'dt_original_display': False,
            }],
            remappings=[
                # pub: old, new
                ('/adora_robot/chassis/frame_info',             '/adora_robot/chassis/frame_info'),
                ('/adora_robot/chassis/drive_left_error_info',  '/adora_robot/chassis/drive_left_error_info'),
                ('/adora_robot/chassis/drive_right_error_info', '/adora_robot/chassis/drive_right_error_info'),
                ('/adora_robot/chassis/auto_charge_info',       '/adora_robot/chassis/auto_charge_info'),
                ('/adora_robot/chassis/battery_info',           '/adora_robot/chassis/battery_info'),
                ('/adora_robot/chassis/date_info',              '/adora_robot/chassis/date_info'),
                ('/adora_robot/chassis/parameter_info',         '/adora_robot/chassis/parameter_info'),
                ('/adora_robot/chassis/state_info',             '/adora_robot/chassis/state_info'),
                ('/adora_robot/chassis/velocity_info',          '/adora_robot/chassis/velocity_info'),
                ('/adora_robot/chassis/hardware_version_info',  '/adora_robot/chassis/hardware_version_info'),
                ('/adora_robot/chassis/led_strip_mode_info',    '/adora_robot/chassis/led_strip_mode_info'),
                ('/adora_robot/chassis/led_strips_info',        '/adora_robot/chassis/led_strips_info'),
                ('/adora_robot/chassis/remote_control_info',    '/adora_robot/chassis/remote_control_info'),
                ('/adora_robot/chassis/software_version_info',  '/adora_robot/chassis/software_version_info'),
                ('/adora_robot/chassis/current_info',           '/adora_robot/chassis/current_info'),
                ('/adora_robot/chassis/speed_info',             '/adora_robot/chassis/speed_info'),
                ('/adora_robot/chassis/odom_info',              '/adora_robot/chassis/odom_info'),

                # sub: old, new
                ('/adora_robot/chassis/velocity_ctrl',       '/adora_robot/chassis/velocity_ctrl'),
                ('/adora_robot/chassis/speed_ctrl',          '/adora_robot/chassis/speed_ctrl'),
                ('/adora_robot/chassis/stop_ctrl',           '/adora_robot/chassis/stop_ctrl'),
                ('/adora_robot/chassis/collision_clean',     '/adora_robot/chassis/collision_clean'),
                ('/adora_robot/chassis/fault_clean',         '/adora_robot/chassis/fault_clean'),
                ('/adora_robot/chassis/auto_charge_ctrl',    '/adora_robot/chassis/auto_charge_ctrl'),
                ('/adora_robot/chassis/led_strip_mode_ctrl', '/adora_robot/chassis/led_strip_mode_ctrl'),
                ('/adora_robot/chassis/led_strip_ctrl',      '/adora_robot/chassis/led_strip_ctrl'),
                ('/adora_robot/chassis/led_strips_ctrl',     '/adora_robot/chassis/led_strips_ctrl'),
                ('/adora_robot/chassis/odom_clean',          '/adora_robot/chassis/odom_clean'),
            ]
        )
    ])
