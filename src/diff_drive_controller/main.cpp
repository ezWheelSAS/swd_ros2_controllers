/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file main.cpp
 */

#include <cstdio>

#include "diff_drive_controller/DiffDriveController.hpp"
#include "rclcpp/rclcpp.hpp"

auto main(int argc, char** argv) -> int
{
    (void)argc;
    (void)argv;

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    auto params_node = std::make_shared<ezw::swd::DiffDriveParameters>("diff_drive_parameters");
    exe.add_node(params_node);

    auto lc_node = std::make_shared<ezw::swd::DiffDriveController>("diff_drive_controller", params_node);
    exe.add_node(lc_node);

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
