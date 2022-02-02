// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swd_ros2_controllers:msg/SafetyFunctions.idl
// generated code does not contain a copyright notice

#ifndef SWD_ROS2_CONTROLLERS__MSG__DETAIL__SAFETY_FUNCTIONS__TRAITS_HPP_
#define SWD_ROS2_CONTROLLERS__MSG__DETAIL__SAFETY_FUNCTIONS__TRAITS_HPP_

#include "swd_ros2_controllers/msg/detail/safety_functions__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const swd_ros2_controllers::msg::SafetyFunctions & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: safe_torque_off
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_torque_off: ";
    value_to_yaml(msg.safe_torque_off, out);
    out << "\n";
  }

  // member: safe_brake_control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_brake_control: ";
    value_to_yaml(msg.safe_brake_control, out);
    out << "\n";
  }

  // member: safety_limited_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_limited_speed: ";
    value_to_yaml(msg.safety_limited_speed, out);
    out << "\n";
  }

  // member: safe_direction_indication_forward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_direction_indication_forward: ";
    value_to_yaml(msg.safe_direction_indication_forward, out);
    out << "\n";
  }

  // member: safe_direction_indication_backward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_direction_indication_backward: ";
    value_to_yaml(msg.safe_direction_indication_backward, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const swd_ros2_controllers::msg::SafetyFunctions & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<swd_ros2_controllers::msg::SafetyFunctions>()
{
  return "swd_ros2_controllers::msg::SafetyFunctions";
}

template<>
inline const char * name<swd_ros2_controllers::msg::SafetyFunctions>()
{
  return "swd_ros2_controllers/msg/SafetyFunctions";
}

template<>
struct has_fixed_size<swd_ros2_controllers::msg::SafetyFunctions>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<swd_ros2_controllers::msg::SafetyFunctions>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<swd_ros2_controllers::msg::SafetyFunctions>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWD_ROS2_CONTROLLERS__MSG__DETAIL__SAFETY_FUNCTIONS__TRAITS_HPP_
