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
        parameters=[{"baseline_m": 0.485}]
    )

    ld.add_action(swd_diff_drive_controller_node)

    return ld
