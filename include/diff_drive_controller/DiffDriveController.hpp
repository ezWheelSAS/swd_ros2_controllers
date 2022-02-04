/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.hpp
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "diff_drive_controller/DiffDriveParameters.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ezw {
    namespace swd {

        /**
         * @brief Differential Drive Controller for ez-Wheel Gen2 wheels
         * It subscribes to the `/node/set_speed` or `/node/cmd_vel` topics:
         * - `/node/set_speed` of type `geometry_msgs::Point`: The `x`, `y`
         *   components of the `/set_speed` message
         *   represents respectively the left and right motor speed in (rad/s)
         * - `/node/cmd_vel` of type `geometry_msgs::Twist`: The linear and angular
         *   velocities.
         * The controller publishes the odometry to `/node/odom` and TFs, the safety
         * functions to `/node/safety`.
        **/
        class DiffDriveController : public rclcpp::Node {
           public:
            /**
             * @brief Construct a new Diff Drive Controller object
             * 
             * @param p_node_name Node name
             * @param p_intra_process_comms  
             */
            explicit DiffDriveController(const std::string &p_node_name, const std::shared_ptr<const DiffDriveParameters> p_params);

            /**
             * @brief 
             * 
             */
            void publish();

           private:
            void cbTopic1();
            void cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr p_cmd_vel);

            rclcpp::TimerBase::SharedPtr m_timer;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_velocity_command_subscriber;

            const std::shared_ptr<const DiffDriveParameters> m_params;
        };

    }  // namespace swd
}  // namespace ezw
