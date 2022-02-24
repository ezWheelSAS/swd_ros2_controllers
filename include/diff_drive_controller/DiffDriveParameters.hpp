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

constexpr auto DEFAULT_PUB_FREQ_HZ = 50;
constexpr auto DEFAULT_MAX_SPEED_RPM = 1050;
constexpr auto DEFAULT_MAX_SLS_RPM = 490;
constexpr auto DEFAULT_WATCHDOG_MS = 1000;
constexpr auto DEFAULT_ODOM_FRAME = "odom";
constexpr auto DEFAULT_BASE_FRAME = "base_link";
constexpr auto DEFAULT_POSITIVE_POLARITY_WHEEL = "Right";
constexpr auto DEFAULT_PUBLISH_ODOM = true;
constexpr auto DEFAULT_PUBLISH_TF = true;
constexpr auto DEFAULT_PUBLISH_SAFETY_FCNS = true;
constexpr auto DEFAULT_BACKWARD_SLS = false;
constexpr auto DEFAULT_LEFT_RELATIVE_ERROR = 0.05;  // 5% of error
constexpr auto DEFAULT_RIGHT_RELATIVE_ERROR = 0.05;
constexpr auto DEFAULT_LEFT_CONFIG_FILE = "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini";
constexpr auto DEFAULT_RIGHT_CONFIG_FILE = "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini";

namespace ezw {
    namespace swd {
        using namespace std::chrono_literals;
        using lg_t = std::lock_guard<std::mutex>;

        class DiffDriveParameters : public rclcpp::Node {
           public:
            /**
             * @brief Construct a new Diff Drive Parameters
             * 
             * @param p_node_name Node name
             */

            explicit DiffDriveParameters(const std::string &p_node_name)
                : Node(p_node_name)
            {
                RCLCPP_INFO(get_logger(), "DiffDriveParameters() is called.");

                // Declare all parameters
                declare_parameter<double>("baseline_m", 0.0);
                declare_parameter<std::string>("left_config_file", DEFAULT_LEFT_CONFIG_FILE);
                declare_parameter<std::string>("right_config_file", DEFAULT_RIGHT_CONFIG_FILE);
                declare_parameter<int>("pub_freq_hz", DEFAULT_PUB_FREQ_HZ);
                declare_parameter<int>("control_timeout_ms", DEFAULT_WATCHDOG_MS);
                declare_parameter<std::string>("base_frame", DEFAULT_BASE_FRAME);
                declare_parameter<std::string>("odom_frame", DEFAULT_ODOM_FRAME);
                declare_parameter<bool>("publish_odom", DEFAULT_PUBLISH_ODOM);
                declare_parameter<bool>("publish_tf", DEFAULT_PUBLISH_TF);
                declare_parameter<bool>("publish_safety_functions", DEFAULT_PUBLISH_SAFETY_FCNS);
                declare_parameter<bool>("have_backward_sls", DEFAULT_BACKWARD_SLS);
                declare_parameter<float>("left_encoder_relative_error", DEFAULT_LEFT_RELATIVE_ERROR);
                declare_parameter<float>("right_encoder_relative_error", DEFAULT_RIGHT_RELATIVE_ERROR);
                declare_parameter<int>("max_speed_rpm", DEFAULT_MAX_SPEED_RPM);
                declare_parameter<int>("safety_limited_speed_rpm", DEFAULT_MAX_SLS_RPM);
                declare_parameter<std::string>("positive_polarity_wheel", DEFAULT_POSITIVE_POLARITY_WHEEL);

                // Read all parameters
                update();

                // Start a timer to update cyclically parameters
                m_timer = create_wall_timer(1000ms, std::bind(&DiffDriveParameters::update, this));
            }

            /**
             * @brief Read all parameters and update
             * 
             */
            void update()
            {
                const lg_t lock(m_mutex);

                // Baseline, distance between wheels (m)
                auto l_baseline_m = m_baseline_m;
                get_parameter("baseline_m", m_baseline_m);
                if (m_baseline_m != l_baseline_m) {
                    RCLCPP_INFO(get_logger(), "Baseline : %f", m_baseline_m);
                }

                // Left config file
                auto l_left_config_file = m_left_config_file;
                get_parameter("left_config_file", m_left_config_file);
                if (m_left_config_file != l_left_config_file) {
                    RCLCPP_INFO(get_logger(), "Left config : %s", m_left_config_file.c_str());
                }

                // Right config file
                auto l_right_config_file = m_right_config_file;
                get_parameter("right_config_file", m_right_config_file);
                if (m_right_config_file != l_right_config_file) {
                    RCLCPP_INFO(get_logger(), "Right config : %s", m_right_config_file.c_str());
                }

                // Pub freq (Hz)
                auto l_pub_freq_hz = m_pub_freq_hz;
                get_parameter("pub_freq_hz", m_pub_freq_hz);
                if (m_pub_freq_hz != l_pub_freq_hz) {
                    RCLCPP_INFO(get_logger(), "Freq (Hz) : %d", m_pub_freq_hz);
                }

                // Watchdog receive (ms)
                auto l_watchdog_receive_ms = m_watchdog_receive_ms;
                get_parameter("control_timeout_ms", m_watchdog_receive_ms);
                if (m_watchdog_receive_ms != l_watchdog_receive_ms) {
                    RCLCPP_INFO(get_logger(), "Watchdog receive (ms): %d", m_watchdog_receive_ms);
                }

                // Base frame
                auto l_base_frame = m_base_frame;
                get_parameter("base_frame", m_base_frame);
                if (m_base_frame != l_base_frame) {
                    RCLCPP_INFO(get_logger(), "Base frame : %s", m_base_frame.c_str());
                }

                // Odom frame
                auto l_odom_frame = m_odom_frame;
                get_parameter("odom_frame", m_odom_frame);
                if (m_odom_frame != l_odom_frame) {
                    RCLCPP_INFO(get_logger(), "Odom frame : %s", m_odom_frame.c_str());
                }

                // Publish odom
                auto l_publish_odom = m_publish_odom;
                get_parameter("publish_odom", m_publish_odom);
                if (m_publish_odom != l_publish_odom) {
                    RCLCPP_INFO(get_logger(), "Publish odom : %d", m_publish_odom);
                }

                // Publish tf
                auto l_publish_tf = m_publish_tf;
                get_parameter("publish_tf", m_publish_tf);
                if (m_publish_tf != l_publish_tf) {
                    RCLCPP_INFO(get_logger(), "Publish tf : %d", m_publish_tf);
                }

                // Publish safety
                auto l_publish_safety = m_publish_safety;
                get_parameter("publish_safety_functions", m_publish_safety);
                if (m_publish_safety != l_publish_safety) {
                    RCLCPP_INFO(get_logger(), "Publish safety : %d", m_publish_safety);
                }

                // Have backward SLS
                auto l_have_backward_sls = m_have_backward_sls;
                get_parameter("have_backward_sls", m_have_backward_sls);
                if (m_have_backward_sls != l_have_backward_sls) {
                    RCLCPP_INFO(get_logger(), "Publish backward SLS : %d", m_have_backward_sls);
                }

                // Left encoder relative error
                auto l_left_encoder_relative_error = m_left_encoder_relative_error;
                get_parameter("left_encoder_relative_error", m_left_encoder_relative_error);
                if (m_left_encoder_relative_error != l_left_encoder_relative_error) {
                    RCLCPP_INFO(get_logger(), "Left encoder relative error : %f", m_left_encoder_relative_error);
                }

                // Right encoder relative error
                auto l_right_encoder_relative_error = m_right_encoder_relative_error;
                get_parameter("right_encoder_relative_error", m_right_encoder_relative_error);
                if (m_right_encoder_relative_error != l_right_encoder_relative_error) {
                    RCLCPP_INFO(get_logger(), "Right encoder relative error : %f", m_right_encoder_relative_error);
                }

                // Max speed rpm
                auto l_max_speed_rpm = max_speed_rpm;
                get_parameter("max_speed_rpm", max_speed_rpm);
                if (max_speed_rpm != l_max_speed_rpm) {
                    RCLCPP_INFO(get_logger(), "Max speed RPM: %d", max_speed_rpm);
                }

                if (max_speed_rpm < 0.) {
                    max_speed_rpm = DEFAULT_MAX_SPEED_RPM;
                    RCLCPP_WARN(get_logger(),
                                "Invalid value %d for parameter 'max_speed_rpm', it should be a positive value. "
                                "Falling back to default (%d)",
                                max_speed_rpm, DEFAULT_MAX_SPEED_RPM);
                }

                // Max speed rpm when SLS enabled
                auto l_max_sls_speed_rpm = max_sls_speed_rpm;
                get_parameter("safety_limited_speed_rpm", max_sls_speed_rpm);
                if (max_sls_speed_rpm != l_max_sls_speed_rpm) {
                    RCLCPP_INFO(get_logger(), "Max SLS speed RPM: %d", max_sls_speed_rpm);
                }

                if (max_sls_speed_rpm < 0.) {
                    max_sls_speed_rpm = DEFAULT_MAX_SLS_RPM;
                    RCLCPP_WARN(get_logger(),
                                "Invalid value %d for parameter 'safety_limited_speed_rpm', it should be a positive value. "
                                "Falling back to default (%d)",
                                max_sls_speed_rpm, DEFAULT_MAX_SLS_RPM);
                }

                // Positive polarity wheel
                auto l_positive_polarity_wheel = positive_polarity_wheel;
                get_parameter("positive_polarity_wheel", positive_polarity_wheel);
                if (positive_polarity_wheel != l_positive_polarity_wheel) {
                    RCLCPP_INFO(get_logger(), "Positive polarity wheel: %s", positive_polarity_wheel.c_str());
                }
            }

            //Getters
            /**
             * @brief Get the Baseline (m)
             * 
             * @return double 
             */
            auto getBaseline() const -> double
            {
                lg_t lock(m_mutex);
                return m_baseline_m;
            }

            /**
             * @brief Get the Left Config File
             * 
             * @return std::string 
             */
            auto getLeftConfigFile() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_left_config_file;
            }

            /**
             * @brief Get the Right Config File
             * 
             * @return std::string 
             */
            auto getRightConfigFile() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_right_config_file;
            }

            /**
             * @brief Get the Pub Freq (Hz)
             * 
             * @return int 
             */
            auto getPubFreqHz() const -> int
            {
                lg_t lock(m_mutex);
                return m_pub_freq_hz;
            }

            /**
             * @brief Get the Watchdog Receive (ms)
             * 
             * @return int 
             */
            auto getWatchdogReceiveMs() const -> int
            {
                lg_t lock(m_mutex);
                return m_watchdog_receive_ms;
            }

            /**
             * @brief Get the Base Frame
             * 
             * @return std::string 
             */
            auto getBaseFrame() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_base_frame;
            }

            /**
             * @brief Get the Odom Frame
             * 
             * @return std::string 
             */
            auto getOdomFrame() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_odom_frame;
            }

            /**
             * @brief Get the Publish Odom
             * 
             * @return true 
             * @return false 
             */
            auto getPublishOdom() const -> bool
            {
                lg_t lock(m_mutex);
                return m_publish_odom;
            }

            /**
             * @brief Get the Publish Tf
             * 
             * @return true 
             * @return false 
             */
            auto getPublishTf() const -> bool
            {
                lg_t lock(m_mutex);
                return m_publish_tf;
            }

            /**
             * @brief Get the Publish Safety
             * 
             * @return true 
             * @return false 
             */
            auto getPublishSafety() const -> bool
            {
                lg_t lock(m_mutex);
                return m_publish_safety;
            }

            /**
             * @brief Get the Have Backward Sls
             * 
             * @return true 
             * @return false 
             */
            auto getHaveBackwardSls() const -> bool
            {
                lg_t lock(m_mutex);
                return m_have_backward_sls;
            }

            /**
             * @brief Get the Left Encoder Relative Error
             * 
             * @return float 
             */
            auto getLeftEncoderRelativeError() const -> float
            {
                lg_t lock(m_mutex);
                return m_left_encoder_relative_error;
            }

            /**
             * @brief Get the Right Encoder Relative Error
             * 
             * @return float 
             */
            auto getRightEncoderRelativeError() const -> float
            {
                lg_t lock(m_mutex);
                return m_right_encoder_relative_error;
            }

            /**
             * @brief Get the Max Speed Rpm
             * 
             * @return int 
             */
            auto getMaxSpeedRpm() const -> int
            {
                lg_t lock(m_mutex);
                return max_speed_rpm;
            }

            /**
             * @brief Get the Max Sls Speed Rpm
             * 
             * @return int 
             */
            auto getMaxSlsSpeedRpm() const -> int
            {
                lg_t lock(m_mutex);
                return max_sls_speed_rpm;
            }

            /**
             * @brief Get the Positive Polarity Wheel
             * 
             * @return std::string 
             */
            auto getPositivePolarityWheel() const -> std::string
            {
                lg_t lock(m_mutex);
                return positive_polarity_wheel;
            }

           private:
            double m_baseline_m;
            std::string m_left_config_file, m_right_config_file;
            int m_pub_freq_hz, m_watchdog_receive_ms;
            std::string m_base_frame, m_odom_frame;
            bool m_publish_odom, m_publish_tf, m_publish_safety, m_have_backward_sls;
            float m_left_encoder_relative_error, m_right_encoder_relative_error;
            int max_speed_rpm, max_sls_speed_rpm;
            std::string positive_polarity_wheel;

            rclcpp::TimerBase::SharedPtr m_timer;
            mutable std::mutex m_mutex;
        };
    }  // namespace swd
}  // namespace ezw
