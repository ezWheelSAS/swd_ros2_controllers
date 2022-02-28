
/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include "diff_drive_controller/DiffDriveController.hpp"

#include <iomanip>
#include <sstream>
#include <string>

#include "ezw-smc-core/Config.hpp"
#include "tf2/LinearMath/Quaternion.h"

#define TIMER_STATE_MACHINE_MS 1000
#define TIMER_SAFETY_MS 200

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ezw::swd {

    /// Convert integer value `val` to text in hexadecimal format.
    /// The minimum width is padded with leading zeros; if not
    /// specified, this `width` is derived from the type of the
    /// argument. Function suitable from char to long long.
    template <typename T>
    inline std::string int_to_hex(T val, size_t width = sizeof(T) * 2)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(width) << std::hex << (val | 0);
        return ss.str();
    }

    DiffDriveController::DiffDriveController(const std::string &p_node_name, const std::shared_ptr<const DiffDriveParameters> p_params) : Node(p_node_name),
                                                                                                                                          m_params(p_params)
    {
        m_tf2_br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publishers
        if (m_params->getPublishOdom()) {
            m_pub_odom = create_publisher<nav_msgs::msg::Odometry>("odom", 5);
        }

        if (m_params->getPublishSafety()) {
            m_pub_safety = create_publisher<swd_ros2_controllers::msg::SafetyFunctions>("safety", 5);
        }

        // Subscribers
        m_sub_brake = create_subscription<std_msgs::msg::Bool>("soft_brake", 5, std::bind(&DiffDriveController::cbSoftBrake, this, _1));
        m_sub_command_set_speed = create_subscription<geometry_msgs::msg::Point>("set_speed", 5, std::bind(&DiffDriveController::cbSetSpeed, this, _1));
        m_sub_command_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 5, std::bind(&DiffDriveController::cbCmdVel, this, _1));

        // Initialize motors
        ezw_error_t err;

        if (m_params->getRightConfigFile().empty()) {
            RCLCPP_ERROR(get_logger(), "Please specify the 'right_swd_config_file' parameter");
            throw std::runtime_error("Please specify the 'right_swd_config_file' parameter");
        }

        // Connect to the right motor

        /* Load motor config file */
        auto lConfig = std::make_shared<ezw::smccore::Config>();
        err = lConfig->load(m_params->getRightConfigFile());
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed loading right motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                         "Config.init() return error code : %d",
                         m_params->getRightConfigFile().c_str(), CON_APP, (int)err);
            throw std::runtime_error("Failed loading right motor's config file");
        }

        m_right_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
        m_r_motor_reduction = lConfig->getReduction();

        /* Init DBus client */
        std::string lHWEntry = lConfig->getHWConfigurationEntry();
        std::transform(lHWEntry.begin(), lHWEntry.end(), lHWEntry.begin(), ::tolower);

        std::string lServiceInstanceName = "commonapi.ezw.smcservice." + lHWEntry;

        /* Init the motor */
        err = m_right_controller.init(lConfig->getContextId(), "local", lServiceInstanceName);
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initializing right motor, EZW_ERR: SMCService : "
                         "Controller::init() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed initializing right motor");
        }

        /* Connect to the motor */
        err = m_right_controller.connect(lConfig->getNodeId());
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initializing right motor, EZW_ERR: SMCService : "
                         "Controller::connect() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed initializing right motor");
        }

        if (m_params->getLeftConfigFile().empty()) {
            RCLCPP_ERROR(get_logger(), "Please specify the 'left_swd_config_file' parameter");
            throw std::runtime_error("Please specify the 'left_swd_config_file' parameter");
        }

        // Connect to the left motor

        /* Load motor config file */
        err = lConfig->load(m_params->getLeftConfigFile());
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed loading left motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                         "Config.init() return error code : %d",
                         m_params->getLeftConfigFile().c_str(), CON_APP, (int)err);
            throw std::runtime_error("Failed initializing left motor");
        }

        m_left_wheel_diameter_m = lConfig->getDiameter() * 1e-3;
        m_l_motor_reduction = lConfig->getReduction();

        /* Init DBus client */
        lHWEntry = lConfig->getHWConfigurationEntry();
        std::transform(lHWEntry.begin(), lHWEntry.end(), lHWEntry.begin(), ::tolower);

        lServiceInstanceName = "commonapi.ezw.smcservice." + lHWEntry;

        /* Init the motor */
        err = m_left_controller.init(lConfig->getContextId(), "local", lServiceInstanceName);
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initializing left motor, EZW_ERR: SMCService : "
                         "Controller::init() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed initializing left motor");
        }

        /* Connect to the motor */
        err = m_left_controller.connect(lConfig->getNodeId());
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initializing left motor, EZW_ERR: SMCService : "
                         "Controller::connect() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed initializing left motor");
        }

        // Read initial encoders values
        err = m_left_controller.getOdometryValue(m_dist_left_prev_mm);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initial reading from left motor, EZW_ERR: SMCService : "
                         "Controller::getOdometryValue() return error code : %d",
                         (int)err);
            throw std::runtime_error("Initial reading from left motor failed");
        }

        err = m_right_controller.getOdometryValue(m_dist_right_prev_mm);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initial reading from right motor, EZW_ERR: SMCService : "
                         "Controller::getOdometryValue() return error code : %d",
                         (int)err);
            throw std::runtime_error("Initial reading from right motor failed");
        }

        // Start the timers
        m_timer_watchdog = create_wall_timer(std::chrono::milliseconds(m_params->getWatchdogReceiveMs()), std::bind(&DiffDriveController::cbTimerWatchdogReceive, this));
        m_timer_pds = create_wall_timer(std::chrono::milliseconds(TIMER_STATE_MACHINE_MS), std::bind(&DiffDriveController::cbTimerStateMachine, this));

        if (m_params->getPublishOdom() || m_params->getPublishTf()) {
            m_timer_odom = create_wall_timer(std::chrono::milliseconds(1000 / m_params->getPubFreqHz()), std::bind(&DiffDriveController::cbTimerOdom, this));
        }

        m_timer_safety = create_wall_timer(std::chrono::milliseconds(TIMER_SAFETY_MS), std::bind(&DiffDriveController::cbTimerSafety, this));

        RCLCPP_INFO(get_logger(), "swd_diff_drive_controller initialized successfully!");
    }

    void DiffDriveController::cbTimerStateMachine()
    {
        static bool m_first_entry = true;

        // NMT state machine
        smccore::INMTService::NMTState nmt_state_l, nmt_state_r;
        smccore::IPDSService::PDSState pds_state_l, pds_state_r;
        ezw_error_t err_l, err_r;

        pds_state_l = pds_state_r = smccore::IPDSService::PDSState::SWITCH_ON_DISABLED;

        err_l = m_left_controller.getNMTState(nmt_state_l);
        err_r = m_right_controller.getNMTState(nmt_state_r);

        if (ERROR_NONE != err_l) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to get the NMT state for left motor, EZW_ERR: SMCService : "
                         "Controller::getPDSState() return error code : %d",
                         (int)err_l);
        }

        if (ERROR_NONE != err_r) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to get the NMT state for right motor, EZW_ERR: SMCService : "
                         "Controller::getPDSState() return error code : %d",
                         (int)err_r);
        }

        bool nmt_ok = (smccore::INMTService::NMTState::OPER == nmt_state_l) && (smccore::INMTService::NMTState::OPER == nmt_state_r);

        if (m_first_entry || m_nmt_ok != nmt_ok) {
            RCLCPP_INFO(get_logger(), "NMT state machine is %s.", nmt_ok ? "OK" : "not OK");
            m_nmt_ok = nmt_ok;
        }

        if (!m_nmt_ok) {
            uint8_t node_id_l, node_id_r;
            std::string can_device_l, can_device_r;

            err_l = m_left_controller.getConnectedNodeId(node_id_l);

            if (ERROR_NONE != err_l) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to set NMT state for left motor, EZW_ERR: SMCService : "
                             "Controller::getConnectedNodeId() return error code : %d",
                             (int)err_l);
            }
            else {
                err_l = m_left_controller.getCanDevice(can_device_l);

                if (ERROR_NONE != err_l) {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to set NMT state for left motor, EZW_ERR: SMCService : "
                                 "Controller::getCanDevice() return error code : %d",
                                 (int)err_l);
                }
                else {
                    std::string cmd = "cansend " + can_device_l + " 000#01" + int_to_hex(node_id_l, 2);
                    system(cmd.c_str());
                }
            }

            err_r = m_right_controller.getConnectedNodeId(node_id_r);

            if (ERROR_NONE != err_r) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to set NMT state for right motor, EZW_ERR: SMCService : "
                             "Controller::getConnectedNodeId() return error code : %d",
                             (int)err_r);
            }
            else {
                err_r = m_right_controller.getCanDevice(can_device_r);

                if (ERROR_NONE != err_r) {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to set NMT state for right motor, EZW_ERR: SMCService : "
                                 "Controller::getCanDevice() return error code : %d",
                                 (int)err_r);
                }
                else {
                    std::string cmd = "cansend " + can_device_r + " 000#01" + int_to_hex(node_id_r, 2);
                    system(cmd.c_str());
                }
            }
        }

        bool pds_ok = false;

        // If NMT is operational, check the PDS state
        if (m_nmt_ok) {
            // PDS state machine
            err_l = m_left_controller.getPDSState(pds_state_l);
            err_r = m_right_controller.getPDSState(pds_state_r);

            if (ERROR_NONE != err_l) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to get the PDS state for left motor, EZW_ERR: SMCService : "
                             "Controller::getPDSState() return error code : %d",
                             (int)err_l);
            }

            if (ERROR_NONE != err_r) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to get the PDS state for right motor, EZW_ERR: SMCService : "
                             "Controller::getPDSState() return error code : %d",
                             (int)err_r);
            }

            pds_ok = (smccore::IPDSService::PDSState::OPERATION_ENABLED == pds_state_l) && (smccore::IPDSService::PDSState::OPERATION_ENABLED == pds_state_r);

            if (smccore::IPDSService::PDSState::OPERATION_ENABLED != pds_state_l) {
                err_l = m_left_controller.enterInOperationEnabledState();
            }

            if (smccore::IPDSService::PDSState::OPERATION_ENABLED != pds_state_r) {
                err_r = m_right_controller.enterInOperationEnabledState();
            }
        }

        if (m_first_entry || m_pds_ok != pds_ok) {
            RCLCPP_INFO(get_logger(), "PDS state machine is %s.", pds_ok ? "OK" : "not OK");
            m_pds_ok = pds_ok;
        }

        m_first_entry = false;
    }

    void DiffDriveController::cbSoftBrake(const std_msgs::msg::Bool &msg)
    {
        // true => Enable brake
        // false => Release brake
        ezw_error_t err = m_left_controller.setHalt(msg.data);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(), "SoftBrake: Failed %s left wheel, EZW_ERR: %d", msg.data ? "braking" : "releasing", (int)err);
        }
        else {
            RCLCPP_INFO(get_logger(), "SoftBrake: Left motor's soft brake %s", msg.data ? "activated" : "disabled");
        }

        err = m_right_controller.setHalt(msg.data);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(), "SoftBrake: Failed %s right wheel, EZW_ERR: %d", msg.data ? "braking" : "releasing", (int)err);
        }
        else {
            RCLCPP_INFO(get_logger(), "SoftBrake: Right motor's soft brake %s", msg.data ? "activated" : "disabled");
        }
    }

    void DiffDriveController::cbTimerOdom()
    {
        nav_msgs::msg::Odometry msg_odom;

        int32_t left_dist_now_mm = 0, right_dist_now_mm = 0;

        ezw_error_t err_l, err_r;
        err_l = m_left_controller.getOdometryValue(left_dist_now_mm);    // In mm
        err_r = m_right_controller.getOdometryValue(right_dist_now_mm);  // In mm

        if (ERROR_NONE != err_l) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading from left motor, EZW_ERR: SMCService : "
                         "Controller::getOdometryValue() return error code : %d",
                         (int)err_l);
            return;
        }

        if (ERROR_NONE != err_r) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading from right motor, EZW_ERR: SMCService : "
                         "Controller::getOdometryValue() return error code : %d",
                         (int)err_r);
            return;
        }

        // Encoder difference between t and t-1
        double d_dist_left_m = static_cast<double>(left_dist_now_mm - m_dist_left_prev_mm) / 1000.0;
        double d_dist_right_m = static_cast<double>(right_dist_now_mm - m_dist_right_prev_mm) / 1000.0;

        // Error calculation (standard deviation)
        double d_dist_left_err_m = m_params->getLeftEncoderRelativeError() * std::abs(d_dist_left_m);
        double d_dist_right_err_m = m_params->getRightEncoderRelativeError() * std::abs(d_dist_right_m);

        rclcpp::Time timestamp = rclcpp::Node::now();

        // Kinematic model
        double d_dist_center = (d_dist_left_m + d_dist_right_m) / 2.0;
        double d_theta = (d_dist_right_m - d_dist_left_m) / m_params->getBaseline();

        // Error propagation (See https://en.wikipedia.org/wiki/Propagation_of_uncertainty#Non-linear_combinations)
        double d_dist_center_err = std::sqrt(std::pow(d_dist_left_err_m / 2.0, 2) + std::pow(d_dist_right_err_m / 2.0, 2));
        double d_theta_err = std::sqrt(std::pow(d_dist_left_err_m / m_params->getBaseline(), 2) + std::pow(d_dist_right_err_m / m_params->getBaseline(), 2));

        // Odometry model, integration of the diff drive kinematic model
        double x_now = m_x_prev + d_dist_center * std::cos(m_theta_prev);
        double y_now = m_y_prev + d_dist_center * std::sin(m_theta_prev);
        double theta_now = M_BOUND_ANGLE(m_theta_prev + d_theta);

        // Error propagation
        double x_now_err = std::sqrt(std::pow(m_x_prev_err, 2) + std::pow(std::cos(m_theta_prev) * d_dist_center_err, 2) + std::pow(-std::sin(m_theta_prev) * d_dist_center * m_theta_prev_err, 2));
        double y_now_err = std::sqrt(std::pow(m_y_prev_err, 2) + std::pow(std::sin(m_theta_prev) * d_dist_center_err, 2) + std::pow(std::cos(m_theta_prev) * d_dist_center * m_theta_prev_err, 2));
        double theta_now_err = std::sqrt(std::pow(m_theta_prev_err, 2) + std::pow(d_theta_err, 2));

        msg_odom.header.stamp = timestamp;
        msg_odom.header.frame_id = m_params->getOdomFrame();
        msg_odom.child_frame_id = m_params->getBaseFrame();

        msg_odom.twist = geometry_msgs::msg::TwistWithCovariance();
        msg_odom.twist.twist.linear.x = d_dist_center * m_params->getPubFreqHz();
        msg_odom.twist.twist.angular.z = d_theta * m_params->getPubFreqHz();

        // Set uncertainties for linear and angular velocities (6 * 6) matrix (x y z Rx Ry Rz)
        msg_odom.twist.covariance[0] = std::pow(d_dist_center_err * m_params->getPubFreqHz(), 2);
        msg_odom.twist.covariance[35] = std::pow(d_theta_err * m_params->getPubFreqHz(), 2);

        msg_odom.pose.pose.position.x = x_now;
        msg_odom.pose.pose.position.y = y_now;
        msg_odom.pose.pose.position.z = 0.0;

        tf2::Quaternion quat_orientation;
        quat_orientation.setRPY(0.0, 0.0, theta_now);
        msg_odom.pose.pose.orientation.x = quat_orientation.getX();
        msg_odom.pose.pose.orientation.y = quat_orientation.getY();
        msg_odom.pose.pose.orientation.z = quat_orientation.getZ();
        msg_odom.pose.pose.orientation.w = quat_orientation.getW();

        // Set uncertainties for x, y, and theta (Rz)
        msg_odom.pose.covariance[0] = std::pow(x_now_err, 2);
        msg_odom.pose.covariance[7] = std::pow(y_now_err, 2);
        msg_odom.pose.covariance[35] = std::pow(theta_now_err, 2);

        if (m_params->getPublishOdom()) {
            m_pub_odom->publish(msg_odom);
        }

        if (m_params->getPublishTf()) {
            geometry_msgs::msg::TransformStamped tf_odom_baselink;
            tf_odom_baselink.header.stamp = timestamp;
            tf_odom_baselink.header.frame_id = m_params->getOdomFrame();
            tf_odom_baselink.child_frame_id = m_params->getBaseFrame();

            tf_odom_baselink.transform.translation.x = msg_odom.pose.pose.position.x;
            tf_odom_baselink.transform.translation.y = msg_odom.pose.pose.position.y;
            tf_odom_baselink.transform.translation.z = msg_odom.pose.pose.position.z;
            tf_odom_baselink.transform.rotation.x = msg_odom.pose.pose.orientation.x;
            tf_odom_baselink.transform.rotation.y = msg_odom.pose.pose.orientation.y;
            tf_odom_baselink.transform.rotation.z = msg_odom.pose.pose.orientation.z;
            tf_odom_baselink.transform.rotation.w = msg_odom.pose.pose.orientation.w;

            // Send TF
            m_tf2_br->sendTransform(tf_odom_baselink);
        }
        m_x_prev = x_now;
        m_y_prev = y_now;
        m_theta_prev = theta_now;
        m_x_prev_err = x_now_err;
        m_y_prev_err = y_now_err;
        m_theta_prev_err = theta_now_err;
        m_dist_left_prev_mm = left_dist_now_mm;
        m_dist_right_prev_mm = right_dist_now_mm;
    }

    void DiffDriveController::cbSetSpeed(const geometry_msgs::msg::Point &speed)
    {
        m_timer_watchdog->reset();

        // Convert rad/s wheel speed to rpm motor speed
        int32_t left = static_cast<int32_t>(speed.x * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
        int32_t right = static_cast<int32_t>(speed.y * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
        RCLCPP_INFO(get_logger(),
                    "Got RightLeftSpeeds command: (left, right) = (%f, %f) rad/s. "
                    "Calculated speeds (left, right) = (%d, %d) rpm",
                    speed->x, speed->y, left, right);
#endif

        setSpeeds(left, right);
    }

    void DiffDriveController::cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr p_cmd_vel)
    {
        m_timer_watchdog->reset();

        double left_vel, right_vel;

        // Control model (diff drive)
        left_vel = (2. * p_cmd_vel->linear.x - p_cmd_vel->angular.z * m_params->getBaseline()) / m_left_wheel_diameter_m;
        right_vel = (2. * p_cmd_vel->linear.x + p_cmd_vel->angular.z * m_params->getBaseline()) / m_right_wheel_diameter_m;

        // Convert rad/s wheel speed to rpm motor speed
        int32_t left = static_cast<int32_t>(left_vel * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
        int32_t right = static_cast<int32_t>(right_vel * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
        RCLCPP_INFO(get_logger(),
                    "Got Twist command: linear = %f m/s, angular = %f rad/s. "
                    "Calculated speeds (left, right) = (%d, %d) rpm",
                    cmd_vel->linear.x, cmd_vel->angular.z, left, right);
#endif

        setSpeeds(left, right);
    }

    void DiffDriveController::setSpeeds(int32_t left_speed, int32_t right_speed)
    {
        // Get the outer wheel speed
        int32_t faster_wheel_speed = M_MAX(std::abs(left_speed), std::abs(right_speed));
        int32_t speed_limit = -1;
        bool max_limited = false;
        bool sls_limited = false;

        // Limit to the maximum allowed speed
        if (faster_wheel_speed > m_params->getMaxSpeedRpm()) {
            speed_limit = m_params->getMaxSpeedRpm();
            max_limited = true;
        }

        // Impose the safety limited speed (SLS) in backward movement when the robot doesn't have backward SLS signal.
        // For example, if it has only one forward-facing safety LiDAR, when the robot move backwards, there's no
        // safety guarantees, hence speed is limited to SLS, otherwise, the safety limit will be decided by the
        // presence of the SLS signal.
        if (!m_params->getHaveBackwardSls() && (left_speed < 0) && (right_speed < 0) && (faster_wheel_speed > m_params->getMaxSlsSpeedRpm())) {
            speed_limit = m_params->getMaxSlsSpeedRpm();
        }

        m_safety_msg_mtx.lock();
        bool sls_signal = m_safety_msg.safety_limited_speed;
        m_safety_msg_mtx.unlock();

        // If SLS detected, impose the safety limited speed (SLS)
        if (sls_signal && (faster_wheel_speed > m_params->getMaxSlsSpeedRpm())) {
            speed_limit = m_params->getMaxSlsSpeedRpm();
            sls_limited = true;
        }

        // The left and right wheels may have different speeds.
        // If we need to limit one of them, we need to scale the second wheel speed.
        // This ensures a speed limitation without distorting the target path.
        if (-1 != speed_limit) {
            // If we enter here, we are sure that (faster_wheel_speed > speed_limit).
            // Get the ratio between the outer (faster) wheel, and the speed limit.
            double speed_ratio = static_cast<double>(speed_limit) / static_cast<double>(faster_wheel_speed);

            // Get the faster wheel
            if (std::abs(left_speed) > std::abs(right_speed)) {
                // Scale right_speed
                right_speed = static_cast<int32_t>(static_cast<double>(right_speed) * speed_ratio);

                // Limit the left_speed
                left_speed = M_SIGN(left_speed) * speed_limit;
            }
            else {
                // Scale left_speed
                left_speed = static_cast<int32_t>(static_cast<double>(left_speed) * speed_ratio);

                // Limit the right_speed
                right_speed = M_SIGN(right_speed) * speed_limit;
            }

            if (max_limited && sls_limited) {
                RCLCPP_INFO(get_logger(),
                            "The target speed exceeds the MAX/SLS maximum speed limit (%d rpm). "
                            "Set speed to (left, right) (%d, %d) rpm",
                            speed_limit, left_speed, right_speed);
            }
            else if (sls_limited) {
                RCLCPP_INFO(get_logger(),
                            "The target speed exceeds the SLS maximum speed limit (%d rpm). "
                            "Set speed to (left, right) (%d, %d) rpm",
                            speed_limit, left_speed, right_speed);
            }
            else if (max_limited) {
                RCLCPP_INFO(get_logger(),
                            "The target speed exceeds the maximum speed limit (%d rpm). "
                            "Set speed to (left, right) (%d, %d) rpm",
                            speed_limit, left_speed, right_speed);
            }
        }

        // If the PDS state is not OPERATION_ENABLED for both wheels, we send a nil speed.
        if (!m_pds_ok) {
            left_speed = right_speed = 0;
        }

        // Send the actual speed (in RPM) to left motor
        ezw_error_t err = m_left_controller.setTargetVelocity(left_speed);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed setting velocity of right motor, EZW_ERR: SMCService : "
                         "Controller::setTargetVelocity() return error code : %d",
                         (int)err);
            return;
        }

        // Send the actual speed (in RPM) to left motor
        err = m_right_controller.setTargetVelocity(right_speed);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed setting velocity of right motor, EZW_ERR: SMCService : "
                         "Controller::setTargetVelocity() return error code : %d",
                         (int)err);
            return;
        }

#if VERBOSE_OUTPUT
        ROS_INFO("Speed sent to motors (left, right) = (%d, %d) rpm", left_speed, right_speed);
#endif
    }

    void DiffDriveController::cbTimerSafety()
    {
        static bool m_first_entry = true;

        swd_ros2_controllers::msg::SafetyFunctions msg;
        ezw_error_t err;
        bool res_l, res_r;

        if (!m_nmt_ok) {
            return;
        }

        msg.header.stamp = rclcpp::Node::now();
        msg.header.frame_id = m_params->getBaseFrame();

        // Reading SBC
        err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1, res_l);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SBC from left motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1, res_r);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SBC from right motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        msg.safe_brake_control = !(res_l || res_r);
        if (m_first_entry || msg.safe_brake_control != m_safety_msg.safe_brake_control) {
            RCLCPP_INFO(get_logger(), msg.safe_brake_control ? "SBC enabled." : "SBC disabled.");
        }

        if (res_l != res_r) {
            RCLCPP_ERROR(get_logger(), "Inconsistant SBC for left and right motors, left=%d, right=%d.", res_l, res_r);
        }

        // Reading STO
        err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::STO, res_l);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading STO from left motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::STO, res_r);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading STO from right motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        msg.safe_torque_off = !(res_l || res_r);
        if (m_first_entry || msg.safe_torque_off != m_safety_msg.safe_torque_off) {
            RCLCPP_INFO(get_logger(), msg.safe_torque_off ? "STO enabled." : "STO disabled.");
        }

        if (res_l != res_r) {
            RCLCPP_ERROR(get_logger(), "Inconsistant STO for left and right motors, left=%d, right=%d.", res_l, res_r);
        }

        // Reading SDI
        bool sdi_l_p, sdi_l_n, sdi_r_p, sdi_r_n, sdi_p, sdi_n;

        err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1, sdi_l_p);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SDI+ from left motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1, sdi_l_n);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SDI- from left motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1, sdi_r_p);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SDI+ from right motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1, sdi_r_n);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SDI- from right motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        if (m_params->getPositivePolarityWheel() == "Left") {
            sdi_p = !(sdi_l_p || sdi_r_n);
            sdi_n = !(sdi_l_n || sdi_r_p);
        }
        else {
            sdi_p = !(sdi_l_n || sdi_r_p);
            sdi_n = !(sdi_l_p || sdi_r_n);
        }

        msg.safe_direction_indication_forward = sdi_p;
        msg.safe_direction_indication_backward = sdi_n;

        if (m_first_entry || msg.safe_direction_indication_forward != m_safety_msg.safe_direction_indication_forward) {
            RCLCPP_INFO(get_logger(), msg.safe_direction_indication_forward ? "SDIp enabled." : "SDIp disabled.");
        }

        if (m_first_entry || msg.safe_direction_indication_backward != m_safety_msg.safe_direction_indication_backward) {
            RCLCPP_INFO(get_logger(), msg.safe_direction_indication_backward ? "SDIn enabled." : "SDIn disabled.");
        }

        // Reading SLS
        err = m_left_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1, res_l);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SLS from left motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        err = m_right_controller.getSafetyFunctionCommand(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1, res_r);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SLS from right motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyFunctionCommand() return error code : %d",
                         (int)err);
        }

        msg.safety_limited_speed = !(res_l || res_r);
        if (m_first_entry || msg.safety_limited_speed != m_safety_msg.safety_limited_speed) {
            RCLCPP_INFO(get_logger(), msg.safety_limited_speed ? "SLS enabled." : "SLS disabled.");
        }

#if VERBOSE_OUTPUT
        ROS_INFO("STO: %d, SDI+: %d, SDI-: %d, SLS: %d", msg.safe_torque_off, msg.safe_direction_indication_forward, msg.safe_direction_indication_backward, msg.safety_limited_speed);
#endif

        m_safety_msg_mtx.lock();
        m_safety_msg = msg;
        m_safety_msg_mtx.unlock();

        if (m_params->getPublishSafety()) {
            m_pub_safety->publish(msg);
        }

        m_first_entry = false;
    }

    void DiffDriveController::cbTimerWatchdogReceive()
    {
        setSpeeds(0, 0);
    }

}  // namespace ezw::swd
