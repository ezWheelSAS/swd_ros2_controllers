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

constexpr auto DEFAULT_BASELINE_M = 0.0;
constexpr auto DEFAULT_LEFT_CONFIG_FILE = "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini";
constexpr auto DEFAULT_RIGHT_CONFIG_FILE = "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini";
constexpr auto DEFAULT_PUB_FREQ_HZ = 20;
constexpr auto DEFAULT_WATCHDOG_RECEIVE_MS = 500;
constexpr auto DEFAULT_BASE_FRAME = "base_link";
constexpr auto DEFAULT_ODOM_FRAME = "odom";
constexpr auto DEFAULT_PUBLISH_ODOM = true;
constexpr auto DEFAULT_PUBLISH_TF = true;
constexpr auto DEFAULT_PUBLISH_SAFETY_FCNS = true;
constexpr auto DEFAULT_MAX_SPEED_RPM = 1050;
constexpr auto DEFAULT_MAX_SLS_RPM = 490;
constexpr auto DEFAULT_HAVE_BACKWARD_SLS = false;
constexpr auto DEFAULT_IS_LEFT_POSITIVE_POLARITY_WHEEL = true;
constexpr auto DEFAULT_LEFT_RELATIVE_ERROR = 0.2;   // 20% of error
constexpr auto DEFAULT_RIGHT_RELATIVE_ERROR = 0.2;  // 20% of error

namespace ezw::swd {
    using namespace std::chrono_literals;
    using lg_t = std::lock_guard<std::mutex>;

    /**
     * @brief This class provides the parameters of the swd_diff_drive_controller node.
     * 
     */
    class DiffDriveParameters : public rclcpp::Node {
       public:
        /**
         * @brief Construct a new Diff Drive Parameters object
         * 
         * @param p_node_name Node name
         */
        explicit DiffDriveParameters(const std::string &p_node_name);

        /**
         * @brief Read and update all parameters
         * 
         */
        void update();

        /**
         * @brief Get the Baseline (m)
         * 
         * @return double 
         */
        auto getBaseline() const -> double;

        /**
         * @brief Get the Left Config File
         * 
         * @return std::string 
         */
        auto getLeftConfigFile() const -> std::string;

        /**
         * @brief Get the Right Config File
         * 
         * @return std::string 
         */
        auto getRightConfigFile() const -> std::string;

        /**
         * @brief Get the Pub Freq (Hz)
         * 
         * @return int 
         */
        auto getPubFreqHz() const -> int;

        /**
         * @brief Get the Watchdog Receive (ms)
         * 
         * @return int 
         */
        auto getWatchdogReceiveMs() const -> int;

        /**
         * @brief Get the Base Frame
         * 
         * @return std::string 
         */
        auto getBaseFrame() const -> std::string;

        /**
         * @brief Get the Odom Frame
         * 
         * @return std::string 
         */
        auto getOdomFrame() const -> std::string;

        /**
         * @brief Get the Publish Odom
         * 
         * @return true if publish enabled
         * @return false 
         */
        auto getPublishOdom() const -> bool;

        /**
         * @brief Get the Publish Tf
         * 
         * @return true if publish enabled
         * @return false 
         */
        auto getPublishTf() const -> bool;

        /**
         * @brief Get the Publish Safety
         * 
         * @return true if publish enabled
         * @return false 
         */
        auto getPublishSafety() const -> bool;

        /**
         * @brief Get the Have Backward SLS
         * 
         * @return true if a backward SLS is handled by a LIDAR sensor
         * @return false 
         */
        auto getHaveBackwardSls() const -> bool;

        /**
         * @brief Get the Left Encoder Relative Error
         * 
         * @return float 
         */
        auto getLeftEncoderRelativeError() const -> float;

        /**
         * @brief Get the Right Encoder Relative Error
         * 
         * @return float 
         */
        auto getRightEncoderRelativeError() const -> float;

        /**
         * @brief Get the Max Speed Rpm
         * 
         * @return int 
         */
        auto getMaxSpeedRpm() const -> int;

        /**
         * @brief Get the Max SLS Speed Rpm
         * 
         * @return int 
         */
        auto getMaxSlsSpeedRpm() const -> int;

        /**
         * @brief Get the Is Left Positive Polarity Wheel
         * 
         * @return bool 
         */
        auto getIsLeftPositivePolarityWheel() const -> bool;

       private:
        /**
         * @brief Parameters modification callback
         * 
         */
        void cbParameters(const rclcpp::Parameter &p_params);

        double m_baseline_m;
        std::string m_left_config_file, m_right_config_file;
        int m_pub_freq_hz, m_watchdog_receive_ms;
        std::string m_base_frame, m_odom_frame;
        bool m_publish_odom, m_publish_tf, m_publish_safety, m_have_backward_sls, m_is_left_positive_polarity_wheel;
        float m_left_encoder_relative_error, m_right_encoder_relative_error;
        int m_max_speed_rpm, m_max_sls_speed_rpm;

        rclcpp::TimerBase::SharedPtr m_timer;
        mutable std::mutex m_mutex;

        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    };
}  // namespace ezw::swd
