
/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveParameters.cpp
 */

#include "diff_drive_controller/DiffDriveParameters.hpp"

namespace ezw::swd {
    DiffDriveParameters::DiffDriveParameters(const std::string &p_node_name)
        : Node(p_node_name, "/parameters")  // Set namespace to "/parameters" to avoid warning "Publisher already registered for provided node name"
    {
        // Declare all parameters
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::FloatingPointRange fpr;
            descriptor.additional_constraints = "Positive value (0.0-999.0)";
            descriptor.type = 3;  // PARAMETER_DOUBLE=3
            descriptor.floating_point_range.resize(1);
            auto &range = descriptor.floating_point_range.at(0);
            range.from_value = 0.001;  // min value
            range.to_value = 999.0;    // max value
            range.step = 0.001;        // precision value

            declare_parameter<double>("baseline_m", DEFAULT_BASELINE_M, descriptor);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            declare_parameter<std::string>("left_config_file", DEFAULT_LEFT_CONFIG_FILE, descriptor);
            declare_parameter<std::string>("right_config_file", DEFAULT_RIGHT_CONFIG_FILE, descriptor);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange fpr;
            descriptor.additional_constraints = "Positive value (0-999)";
            descriptor.type = 2;  // PARAMETER_INTEGER=2
            descriptor.integer_range.resize(1);
            auto &range = descriptor.integer_range.at(0);
            range.from_value = 1;  // min value
            range.to_value = 999;  // max value
            range.step = 1;        // precision value

            declare_parameter<int>("pub_freq_hz", DEFAULT_PUB_FREQ_HZ, descriptor);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange fpr;
            descriptor.additional_constraints = "Positive value (0-9999)";
            descriptor.type = 2;  // PARAMETER_INTEGER=2
            descriptor.integer_range.resize(1);
            auto &range = descriptor.integer_range.at(0);
            range.from_value = 1;   // min value
            range.to_value = 9999;  // max value
            range.step = 1;         // precision value
            descriptor.read_only = true;
            declare_parameter<int>("watchdog_receive_ms", DEFAULT_WATCHDOG_RECEIVE_MS, descriptor);
        }
        declare_parameter<std::string>("base_frame", DEFAULT_BASE_FRAME);
        declare_parameter<std::string>("odom_frame", DEFAULT_ODOM_FRAME);
        declare_parameter<bool>("publish_odom", DEFAULT_PUBLISH_ODOM);
        declare_parameter<bool>("publish_tf", DEFAULT_PUBLISH_TF);
        declare_parameter<bool>("publish_safety_functions", DEFAULT_PUBLISH_SAFETY_FCNS);
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::IntegerRange fpr;
            descriptor.additional_constraints = "Positive value (50-2000)";
            descriptor.type = 2;  // PARAMETER_INTEGER=2
            descriptor.integer_range.resize(1);
            auto &range = descriptor.integer_range.at(0);
            range.from_value = 50;  // min value
            range.to_value = 2000;  // max value
            range.step = 1;         // precision value

            declare_parameter<int>("max_speed_rpm", DEFAULT_MAX_SPEED_RPM, descriptor);
            declare_parameter<int>("safety_limited_speed_rpm", DEFAULT_MAX_SLS_RPM, descriptor);
        }
        declare_parameter<bool>("have_backward_sls", DEFAULT_HAVE_BACKWARD_SLS);
        declare_parameter<bool>("is_left_positive_polarity_wheel", DEFAULT_IS_LEFT_POSITIVE_POLARITY_WHEEL);
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            rcl_interfaces::msg::FloatingPointRange fpr;
            descriptor.additional_constraints = "Percent value (0.0-1.0)";
            descriptor.type = 3;  // PARAMETER_DOUBLE=3
            descriptor.floating_point_range.resize(1);
            auto &range = descriptor.floating_point_range.at(0);
            range.from_value = 0.01;  // min value
            range.to_value = 1.0;     // max value
            range.step = 0.01;        // precision value

            declare_parameter<float>("left_encoder_relative_error", DEFAULT_LEFT_RELATIVE_ERROR, descriptor);
            declare_parameter<float>("right_encoder_relative_error", DEFAULT_RIGHT_RELATIVE_ERROR, descriptor);
        }

        // Read all parameters
        update();

        // Start a timer to update cyclically parameters
        m_timer = create_wall_timer(1000ms, std::bind(&DiffDriveParameters::update, this));
    }

    void DiffDriveParameters::update()
    {
        const lg_t lock(m_mutex);

        // Baseline, distance between wheels (m)
        auto l_baseline_m = m_baseline_m;
        get_parameter("baseline_m", m_baseline_m);
        if (m_baseline_m != l_baseline_m) {
            RCLCPP_INFO(get_logger(), "baseline_m : %f", m_baseline_m);
        }

        // Left config file
        auto l_left_config_file = m_left_config_file;
        get_parameter("left_config_file", m_left_config_file);
        if (m_left_config_file != l_left_config_file) {
            RCLCPP_INFO(get_logger(), "left_config_file : %s", m_left_config_file.c_str());
        }

        // Right config file
        auto l_right_config_file = m_right_config_file;
        get_parameter("right_config_file", m_right_config_file);
        if (m_right_config_file != l_right_config_file) {
            RCLCPP_INFO(get_logger(), "right_config_file : %s", m_right_config_file.c_str());
        }

        // Pub freq (Hz)
        auto l_pub_freq_hz = m_pub_freq_hz;
        get_parameter("pub_freq_hz", m_pub_freq_hz);
        if (m_pub_freq_hz != l_pub_freq_hz) {
            RCLCPP_INFO(get_logger(), "pub_freq_hz : %d", m_pub_freq_hz);
        }

        // Watchdog receive (ms)
        auto l_watchdog_receive_ms = m_watchdog_receive_ms;
        get_parameter("watchdog_receive_ms", m_watchdog_receive_ms);
        if (m_watchdog_receive_ms != l_watchdog_receive_ms) {
            RCLCPP_INFO(get_logger(), "watchdog_receive_ms : %d", m_watchdog_receive_ms);
        }

        // Base frame
        auto l_base_frame = m_base_frame;
        get_parameter("base_frame", m_base_frame);
        if (m_base_frame != l_base_frame) {
            RCLCPP_INFO(get_logger(), "base_frame : %s", m_base_frame.c_str());
        }

        // Odom frame
        auto l_odom_frame = m_odom_frame;
        get_parameter("odom_frame", m_odom_frame);
        if (m_odom_frame != l_odom_frame) {
            RCLCPP_INFO(get_logger(), "odom_frame : %s", m_odom_frame.c_str());
        }

        // Publish odom
        auto l_publish_odom = m_publish_odom;
        get_parameter("publish_odom", m_publish_odom);
        if (m_publish_odom != l_publish_odom) {
            RCLCPP_INFO(get_logger(), "publish_odom : %d", m_publish_odom);
        }

        // Publish tf
        auto l_publish_tf = m_publish_tf;
        get_parameter("publish_tf", m_publish_tf);
        if (m_publish_tf != l_publish_tf) {
            RCLCPP_INFO(get_logger(), "publish_tf : %d", m_publish_tf);
        }

        // Publish safety
        auto l_publish_safety = m_publish_safety;
        get_parameter("publish_safety_functions", m_publish_safety);
        if (m_publish_safety != l_publish_safety) {
            RCLCPP_INFO(get_logger(), "publish_safety_functions : %d", m_publish_safety);
        }

        // Have backward SLS
        auto l_have_backward_sls = m_have_backward_sls;
        get_parameter("have_backward_sls", m_have_backward_sls);
        if (m_have_backward_sls != l_have_backward_sls) {
            RCLCPP_INFO(get_logger(), "have_backward_sls : %d", m_have_backward_sls);
        }

        // Left encoder relative error
        auto l_left_encoder_relative_error = m_left_encoder_relative_error;
        get_parameter("left_encoder_relative_error", m_left_encoder_relative_error);
        if (m_left_encoder_relative_error != l_left_encoder_relative_error) {
            RCLCPP_INFO(get_logger(), "left_encoder_relative_error : %f", m_left_encoder_relative_error);
        }

        // Right encoder relative error
        auto l_right_encoder_relative_error = m_right_encoder_relative_error;
        get_parameter("right_encoder_relative_error", m_right_encoder_relative_error);
        if (m_right_encoder_relative_error != l_right_encoder_relative_error) {
            RCLCPP_INFO(get_logger(), "right_encoder_relative_error : %f", m_right_encoder_relative_error);
        }

        // Max speed rpm
        auto l_max_speed_rpm = m_max_speed_rpm;
        get_parameter("max_speed_rpm", m_max_speed_rpm);
        if (m_max_speed_rpm != l_max_speed_rpm) {
            RCLCPP_INFO(get_logger(), "max_speed_rpm : %d", m_max_speed_rpm);
        }

        // Max speed rpm when SLS enabled
        auto l_max_sls_speed_rpm = m_max_sls_speed_rpm;
        get_parameter("safety_limited_speed_rpm", m_max_sls_speed_rpm);
        if (m_max_sls_speed_rpm != l_max_sls_speed_rpm) {
            RCLCPP_INFO(get_logger(), "safety_limited_speed_rpm : %d", m_max_sls_speed_rpm);
        }

        // Is Left Positive polarity wheel
        auto l_is_left_positive_polarity_wheel = m_is_left_positive_polarity_wheel;
        get_parameter("is_left_positive_polarity_wheel", m_is_left_positive_polarity_wheel);
        if (m_is_left_positive_polarity_wheel != l_is_left_positive_polarity_wheel) {
            RCLCPP_INFO(get_logger(), "is_left_positive_polarity_wheel : %d", m_is_left_positive_polarity_wheel);
        }
    }

    auto DiffDriveParameters::getBaseline() const -> double
    {
        lg_t lock(m_mutex);
        return m_baseline_m;
    }

    auto DiffDriveParameters::getLeftConfigFile() const -> std::string
    {
        lg_t lock(m_mutex);
        return m_left_config_file;
    }

    auto DiffDriveParameters::getRightConfigFile() const -> std::string
    {
        lg_t lock(m_mutex);
        return m_right_config_file;
    }

    auto DiffDriveParameters::getPubFreqHz() const -> int
    {
        lg_t lock(m_mutex);
        return m_pub_freq_hz;
    }

    auto DiffDriveParameters::getWatchdogReceiveMs() const -> int
    {
        lg_t lock(m_mutex);
        return m_watchdog_receive_ms;
    }

    auto DiffDriveParameters::getBaseFrame() const -> std::string
    {
        lg_t lock(m_mutex);
        return m_base_frame;
    }

    auto DiffDriveParameters::getOdomFrame() const -> std::string
    {
        lg_t lock(m_mutex);
        return m_odom_frame;
    }

    auto DiffDriveParameters::getPublishOdom() const -> bool
    {
        lg_t lock(m_mutex);
        return m_publish_odom;
    }

    auto DiffDriveParameters::getPublishTf() const -> bool
    {
        lg_t lock(m_mutex);
        return m_publish_tf;
    }

    auto DiffDriveParameters::getPublishSafety() const -> bool
    {
        lg_t lock(m_mutex);
        return m_publish_safety;
    }

    auto DiffDriveParameters::getHaveBackwardSls() const -> bool
    {
        lg_t lock(m_mutex);
        return m_have_backward_sls;
    }

    auto DiffDriveParameters::getLeftEncoderRelativeError() const -> float
    {
        lg_t lock(m_mutex);
        return m_left_encoder_relative_error;
    }

    auto DiffDriveParameters::getRightEncoderRelativeError() const -> float
    {
        lg_t lock(m_mutex);
        return m_right_encoder_relative_error;
    }

    auto DiffDriveParameters::getMaxSpeedRpm() const -> int
    {
        lg_t lock(m_mutex);
        return m_max_speed_rpm;
    }

    auto DiffDriveParameters::getMaxSlsSpeedRpm() const -> int
    {
        lg_t lock(m_mutex);
        return m_max_sls_speed_rpm;
    }

    auto DiffDriveParameters::getIsLeftPositivePolarityWheel() const -> bool
    {
        lg_t lock(m_mutex);
        return m_is_left_positive_polarity_wheel;
    }
}  // namespace ezw::swd
