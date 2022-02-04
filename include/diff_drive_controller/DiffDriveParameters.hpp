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
            }

            void update()
            {
                const lg_t lock(m_mutex);

                get_parameter("parameter1", m_parameter1);
                RCLCPP_INFO(get_logger(), "Parameter1 : %s", m_parameter1.c_str());
            }

            auto getParameter1() const -> std::string
            {
                lg_t lock(m_mutex);
                return m_parameter1;
            }

           private:
            std::string m_parameter1;

            rclcpp::TimerBase::SharedPtr m_timer;
            mutable std::mutex m_mutex;
        };
    }  // namespace swd
}  // namespace ezw
