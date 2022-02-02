// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swd_ros2_controllers:msg/SafetyFunctions.idl
// generated code does not contain a copyright notice

#ifndef SWD_ROS2_CONTROLLERS__MSG__DETAIL__SAFETY_FUNCTIONS__STRUCT_HPP_
#define SWD_ROS2_CONTROLLERS__MSG__DETAIL__SAFETY_FUNCTIONS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__swd_ros2_controllers__msg__SafetyFunctions __attribute__((deprecated))
#else
# define DEPRECATED__swd_ros2_controllers__msg__SafetyFunctions __declspec(deprecated)
#endif

namespace swd_ros2_controllers
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SafetyFunctions_
{
  using Type = SafetyFunctions_<ContainerAllocator>;

  explicit SafetyFunctions_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safe_torque_off = false;
      this->safe_brake_control = false;
      this->safety_limited_speed = false;
      this->safe_direction_indication_forward = false;
      this->safe_direction_indication_backward = false;
    }
  }

  explicit SafetyFunctions_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safe_torque_off = false;
      this->safe_brake_control = false;
      this->safety_limited_speed = false;
      this->safe_direction_indication_forward = false;
      this->safe_direction_indication_backward = false;
    }
  }

  // field types and members
  using _safe_torque_off_type =
    bool;
  _safe_torque_off_type safe_torque_off;
  using _safe_brake_control_type =
    bool;
  _safe_brake_control_type safe_brake_control;
  using _safety_limited_speed_type =
    bool;
  _safety_limited_speed_type safety_limited_speed;
  using _safe_direction_indication_forward_type =
    bool;
  _safe_direction_indication_forward_type safe_direction_indication_forward;
  using _safe_direction_indication_backward_type =
    bool;
  _safe_direction_indication_backward_type safe_direction_indication_backward;

  // setters for named parameter idiom
  Type & set__safe_torque_off(
    const bool & _arg)
  {
    this->safe_torque_off = _arg;
    return *this;
  }
  Type & set__safe_brake_control(
    const bool & _arg)
  {
    this->safe_brake_control = _arg;
    return *this;
  }
  Type & set__safety_limited_speed(
    const bool & _arg)
  {
    this->safety_limited_speed = _arg;
    return *this;
  }
  Type & set__safe_direction_indication_forward(
    const bool & _arg)
  {
    this->safe_direction_indication_forward = _arg;
    return *this;
  }
  Type & set__safe_direction_indication_backward(
    const bool & _arg)
  {
    this->safe_direction_indication_backward = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator> *;
  using ConstRawPtr =
    const swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swd_ros2_controllers__msg__SafetyFunctions
    std::shared_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swd_ros2_controllers__msg__SafetyFunctions
    std::shared_ptr<swd_ros2_controllers::msg::SafetyFunctions_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SafetyFunctions_ & other) const
  {
    if (this->safe_torque_off != other.safe_torque_off) {
      return false;
    }
    if (this->safe_brake_control != other.safe_brake_control) {
      return false;
    }
    if (this->safety_limited_speed != other.safety_limited_speed) {
      return false;
    }
    if (this->safe_direction_indication_forward != other.safe_direction_indication_forward) {
      return false;
    }
    if (this->safe_direction_indication_backward != other.safe_direction_indication_backward) {
      return false;
    }
    return true;
  }
  bool operator!=(const SafetyFunctions_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SafetyFunctions_

// alias to use template instance with default allocator
using SafetyFunctions =
  swd_ros2_controllers::msg::SafetyFunctions_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace swd_ros2_controllers

#endif  // SWD_ROS2_CONTROLLERS__MSG__DETAIL__SAFETY_FUNCTIONS__STRUCT_HPP_
