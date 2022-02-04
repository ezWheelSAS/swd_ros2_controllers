/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveParameters.hpp
 */

#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#define DEFAULT_PUB_FREQ_HZ 50
#define DEFAULT_MAX_WHEEL_SPEED_RPM 75.0  // 75 rpm Wheel => Motor (75 * 14 = 1050 rpm)
#define DEFAULT_MAX_SLS_WHEEL_RPM 30.0    // 30 rpm Wheel => Motor (30 * 14 = 490 rpm)
#define DEFAULT_WATCHDOG_MS 1000
#define DEFAULT_ODOM_FRAME std::string("odom")
#define DEFAULT_BASE_FRAME std::string("base_link")
#define DEFAULT_POSITIVE_POLARITY_WHEEL std::string("Right")
#define DEFAULT_CTRL_MODE std::string("Twist")
#define DEFAULT_PUBLISH_ODOM true
#define DEFAULT_PUBLISH_TF true
#define DEFAULT_PUBLISH_SAFETY_FCNS true
#define DEFAULT_BACKWARD_SLS false
#define DEFAULT_LEFT_RELATIVE_ERROR 0.05  // 5% of error
#define DEFAULT_RIGHT_RELATIVE_ERROR 0.05

using namespace std::chrono_literals;
using lg_t = std::lock_guard<std::mutex>;

namespace ezw {
    namespace swd {
        class DiffDriveParameters : public rclcpp::Node {
           public:
            explicit DiffDriveParameters(const std::string &p_node_name)
                : Node(p_node_name)
            {
                declare_parameter<std::string>("parameter1", "world");
                m_timer = create_wall_timer(
                    1000ms, std::bind(&DiffDriveParameters::update, this));

                declare_parameter<double>("baseline_m", 0.0);
                declare_parameter<std::string>("left_config_file", "");
                declare_parameter<std::string>("right_config_file", "");
                declare_parameter<float>("pub_freq_hz", DEFAULT_PUB_FREQ_HZ);
                declare_parameter<float>("control_timeout_ms", DEFAULT_WATCHDOG_MS);
                declare_parameter<std::string>("base_frame", DEFAULT_BASE_FRAME);
                declare_parameter<std::string>("odom_frame", DEFAULT_ODOM_FRAME);
                declare_parameter<bool>("publish_odom", DEFAULT_PUBLISH_ODOM);
                declare_parameter<bool>("publish_tf", DEFAULT_PUBLISH_TF);
                declare_parameter<bool>("publish_safety_functions", DEFAULT_PUBLISH_SAFETY_FCNS);
                declare_parameter<bool>("have_backward_sls", DEFAULT_BACKWARD_SLS);
                declare_parameter<float>("left_encoder_relative_error", DEFAULT_LEFT_RELATIVE_ERROR);
                declare_parameter<float>("right_encoder_relative_error", DEFAULT_RIGHT_RELATIVE_ERROR);
                declare_parameter<float>("wheel_max_speed_rpm", DEFAULT_MAX_WHEEL_SPEED_RPM);
                declare_parameter<float>("wheel_safety_limited_speed_rpm", DEFAULT_MAX_SLS_WHEEL_RPM);
                declare_parameter<std::string>("positive_polarity_wheel", DEFAULT_POSITIVE_POLARITY_WHEEL);
                declare_parameter<std::string>("control_mode", DEFAULT_CTRL_MODE);
            }

            void update()
            {
                const lg_t lock(m_mutex);

                get_parameter("parameter1", m_parameter1);
                RCLCPP_INFO(get_logger(), "Parameter1 : %s", m_parameter1.c_str());

                get_parameter("positive_polarity_wheel", positive_polarity_wheel);
                RCLCPP_INFO(get_logger(), "Left : %s", positive_polarity_wheel.c_str());
            }

            auto getParameter1() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_parameter1;
            }

            auto getBaseline() const -> double
            {
                lg_t lock(m_mutex);
                return m_baseline_m;
            }

            auto getLeftConfigFile() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_left_config_file;
            }

            auto getRightConfigFile() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_right_config_file;
            }

            auto getPubFreqHz() const -> float
            {
                lg_t lock(m_mutex);
                return m_pub_freq_hz;
            }

            auto getWatchdogReceiveMs() const -> float
            {
                lg_t lock(m_mutex);
                return m_watchdog_receive_ms;
            }

            auto getBaseFrame() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_base_frame;
            }

            auto getOdomFrame() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_odom_frame;
            }

            auto getPublishOdom() const -> bool
            {
                lg_t lock(m_mutex);
                return m_publish_odom;
            }

            auto getPublishTf() const -> bool
            {
                lg_t lock(m_mutex);
                return m_publish_tf;
            }

            auto getPublishSafety() const -> bool
            {
                lg_t lock(m_mutex);
                return m_publish_safety;
            }

            auto getHaveBackwardSls() const -> bool
            {
                lg_t lock(m_mutex);
                return m_have_backward_sls;
            }

            auto getLeftEncoderRelativeError() const -> float
            {
                lg_t lock(m_mutex);
                return m_left_encoder_relative_error;
            }

            auto getRightEncoderRelativeError() const -> float
            {
                lg_t lock(m_mutex);
                return m_right_encoder_relative_error;
            }

            auto getMaxWheelSpeedRpm() const -> double
            {
                lg_t lock(m_mutex);
                return max_wheel_speed_rpm;
            }

            auto getMawSlsWheelSpeedRpm() const -> double
            {
                lg_t lock(m_mutex);
                return max_sls_wheel_speed_rpm;
            }

            auto getPositivePolarityWheel() const -> std::string
            {
                lg_t lock(m_mutex);
                return positive_polarity_wheel;
            }

            auto getCtrlMode() const -> std::string
            {
                lg_t lock(m_mutex);
                return ctrl_mode;
            }

           private:
            std::string m_parameter1;
            double m_baseline_m;
            std::string m_left_config_file;
            std::string m_right_config_file;
            float m_pub_freq_hz;
            float m_watchdog_receive_ms;
            std::string m_base_frame;
            std::string m_odom_frame;
            bool m_publish_odom;
            bool m_publish_tf;
            bool m_publish_safety;
            bool m_have_backward_sls;
            float m_left_encoder_relative_error;
            float m_right_encoder_relative_error;
            double max_wheel_speed_rpm;
            double max_sls_wheel_speed_rpm;
            std::string positive_polarity_wheel;
            std::string ctrl_mode;

            rclcpp::TimerBase::SharedPtr m_timer;
            mutable std::mutex m_mutex;
        };
    }  // namespace swd
}  // namespace ezw
