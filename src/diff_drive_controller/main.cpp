#include <cstdio>

#include "diff_drive_controller/DiffDriveController.hpp"
#include "diff_drive_controller/LifeCycleTalker.hpp"

auto main(int argc, char** argv) -> int
{
    (void)argc;
    (void)argv;

    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto lc_node = std::make_shared<ezw::swd::LifecycleTalker>("diff_drive_talker");

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
