
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

#include "msg/safety_functions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace {
    constexpr auto DEFAULT_COMMAND_TOPIC = "cmd_vel";
}

namespace ezw {
    namespace swd {
        DiffDriveController::DiffDriveController(const std::string &p_node_name, const std::shared_ptr<const DiffDriveParameters> p_params) : Node(p_node_name),
                                                                                                                                              m_params(p_params)
        {
            RCLCPP_INFO(get_logger(), "on_configure() is called.");

            //Publisher
            m_publisher = create_publisher<std_msgs::msg::String>("topic1", 5);
            m_timer = create_wall_timer(500ms, std::bind(&DiffDriveController::cbTopic1, this));

            //Subscriber
            m_velocity_command_subscriber = create_subscription<geometry_msgs::msg::Twist>(DEFAULT_COMMAND_TOPIC, 5, std::bind(&DiffDriveController::cbCmdVel, this, _1));

            auto parameter1 = m_params->getParameter1();
        }  // namespace swd

        void DiffDriveController::cbTopic1()
        {
            static auto count = 0;
            auto message = std_msgs::msg::String();
            message.data = "topic1 : " + std::to_string(count++);
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
            m_publisher->publish(message);
        }

        ///
        /// \brief Change robot velocity (linear [m/s], angular [rad/s])
        ///
        void DiffDriveController::cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr p_cmd_vel)
        {
            RCLCPP_INFO(get_logger(), "Got Twist command: linear = %f m/s, angular = %f rad/s. ", p_cmd_vel->linear.x, p_cmd_vel->angular.z);
        }

    }  // namespace swd
}  // namespace ezw