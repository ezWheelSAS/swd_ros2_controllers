/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.hpp
 */

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "diff_drive_controller/DiffDriveParameters.hpp"
#include "ezw-smc-service/DBusClient.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "swd_ros2_controllers/msg/safety_functions.hpp"

#define M_MAX(a, b) ((a) > (b) ? (a) : (b))
#define M_MIN(a, b) ((a) < (b) ? (a) : (b))
#define M_SIGN(a) ((a) > 0 ? 1 : -1)
#define M_BOUND_ANGLE(a) (((a) > M_PI) ? ((a)-2. * M_PI) : (((a) < -M_PI) ? ((a) + 2. * M_PI) : (a)))

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
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_odom;
            rclcpp::Publisher<swd_ros2_controllers::msg::SafetyFunctions>::SharedPtr m_pub_safety;
            //rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_command;
            //rclcpp::Subscription<std_msgs::msg::Bool::ConstPtr> m_sub_brake;

            std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf2_br;

            void cbTopic1();
            void cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr p_cmd_vel);
            void cbTimerOdom(), cbWatchdog(), cbTimerStateMachine(), cbTimerSafety();
            void cbSoftBrake(const std_msgs::msg::Bool::ConstPtr &msg);
            void cbSetSpeed(const geometry_msgs::msg::Point &speed);
            void setSpeeds(int32_t left_speed, int32_t right_speed);

            rclcpp::TimerBase::SharedPtr m_timer;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_velocity_command_subscriber;
            rclcpp::TimerBase::SharedPtr m_timer_odom, m_timer_watchdog, m_timer_pds, m_timer_safety;

            std::mutex m_safety_msg_mtx;
            swd_ros2_controllers::msg::SafetyFunctions m_safety_msg;

            double m_x_prev = 0.0, m_y_prev = 0.0, m_theta_prev = 0.0;
            double m_x_prev_err = 0.0, m_y_prev_err = 0.0, m_theta_prev_err = 0.0;
            int32_t m_dist_left_prev_mm = 0, m_dist_right_prev_mm = 0;

            const std::shared_ptr<const DiffDriveParameters> m_params;

            //Params
            double m_baseline_m, m_left_wheel_diameter_m, m_right_wheel_diameter_m, m_l_motor_reduction, m_r_motor_reduction, m_left_encoder_relative_error, m_right_encoder_relative_error;
            int m_left_wheel_polarity, m_max_motor_speed_rpm, m_motor_sls_rpm;
            bool m_have_backward_sls, m_publish_odom, m_publish_tf, m_publish_safety, m_nmt_ok, m_pds_ok;
            ezw::smcservice::DBusClient m_left_controller, m_right_controller;
        };

    }  // namespace swd
}  // namespace ezw
