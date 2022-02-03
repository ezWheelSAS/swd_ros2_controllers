/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveParameters.hpp
 */

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

namespace ezw {
    namespace swd {
        class DiffDriveParameters : public rclcpp::Node {
           public:
            explicit DiffDriveParameters(const std::string &p_node_name)
                : Node(p_node_name)
            {
                declare_parameter<std::string>("parameter1", "world");
                m_timer = create_wall_timer(
                    1000ms, std::bind(&DiffDriveParameters::respond, this));
            }

            void respond()
            {
                get_parameter("parameter1", m_parameter1);
                RCLCPP_INFO(get_logger(), "Parameter1 : %s", m_parameter1.c_str());
            }

            auto getTimer() -> rclcpp::TimerBase::SharedPtr
            {
                return m_timer;
            }

            void setTimer(rclcpp::TimerBase::SharedPtr p_timer)
            {
                m_timer = p_timer;
            }

            auto getParameter1() const -> std::string
            {
                return m_parameter1;
            }

            void setParameter1(std::string p_parameter1)
            {
                m_parameter1 = p_parameter1;
            }

           private:
            std::string m_parameter1;

            rclcpp::TimerBase::SharedPtr m_timer;
        };
    }  // namespace swd
}  // namespace ezw
