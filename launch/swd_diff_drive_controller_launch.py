################################################
## Copyright (C) 2022 ez-Wheel S.A.S.         ##
##                                            ##
## @file swd_diff_drive_controller_launch.py  ##
################################################


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    swd_diff_drive_controller_node = Node(
        package='swd_ros2_controllers',
        executable='swd_diff_drive_controller',
        name='swd_diff_drive_controller',
        parameters=[{"baseline_m": 0.485},
                    {"pub_freq_hz": 20},
                    {"left_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini"},
                    {"right_swd_config_file": "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini"},
                    {"command_timeout_ms": 500},
                    {"control_mode": "Twist"},
                    {"base_frame": "base_link"},
                    {"odom_frame": "odom"},
                    {"positive_polarity_wheel": "Right"},
                    {"wheel_max_speed_rpm": 95.0},
                    {"wheel_safety_limited_speed_rpm": 40.0},
                    {"have_backward_sls": False},
                    {"left_encoder_relative_error": 0.2},
                    {"right_encoder_relative_error": 0.2},
                    {"publish_odom": True},
                    {"publish_tf": True},
                    {"publish_safety_functions": True}]
    )

    ld.add_action(swd_diff_drive_controller_node)

    return ld
