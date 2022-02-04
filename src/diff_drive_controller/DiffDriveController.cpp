
/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include "diff_drive_controller/DiffDriveController.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "msg/safety_functions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ezw {
    namespace swd {
        DiffDriveController::DiffDriveController(const std::string &p_node_name, const std::shared_ptr<const DiffDriveParameters> p_params, bool p_intra_process_comms) : rclcpp_lifecycle::LifecycleNode(p_node_name, rclcpp::NodeOptions().use_intra_process_comms(p_intra_process_comms)),
                                                                                                                                                                          m_params(p_params)
        {
        }

        void DiffDriveController::publish()
        {
            // static size_t count = 0;
            // auto msg = std::make_unique<std_msgs::msg::String>();
            // msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);

            // // Print the current state for demo purposes
            // if (!m_pub->is_activated()) {
            //     RCLCPP_INFO(
            //         get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
            // }
            // else {
            //     RCLCPP_INFO(
            //         get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
            // }

            // // We independently from the current state call publish on the lifecycle
            // // publisher.
            // // Only if the publisher is in an active state, the message transfer is
            // // enabled and the message actually published.
            // m_pub->publish(std::move(msg));
            ;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DiffDriveController::on_configure(const rclcpp_lifecycle::State &)
        {
            // This callback is supposed to be used for initialization and
            // configuring purposes.
            // We thus initialize and configure our publishers and timers.
            // The lifecycle node API does return lifecycle components such as
            // lifecycle publishers. These entities obey the lifecycle and
            // can comply to the current state of the node.
            // As of the beta version, there is only a lifecycle publisher
            // available.

            // m_pub = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
            // m_timer = this->create_wall_timer(
            //     1s, std::bind(&DiffDriveController::publish, this));

            // TODO(LDA) : 4 timers

            RCLCPP_INFO(get_logger(), "on_configure() is called.");

            auto parameter1 = m_params->getParameter1();
            RCLCPP_INFO(get_logger(), "lParameter1 : %s", parameter1.c_str());

            // We return a success and hence invoke the transition to the next
            // step: "inactive".
            // If we returned TRANSITION_CALLBACK_FAILURE instead, the state machine
            // would stay in the "unconfigured" state.
            // In case of TRANSITION_CALLBACK_ERROR or any thrown exception within
            // this callback, the state machine transitions to state "errorprocessing".
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
    }  // namespace swd
}  // namespace ezw