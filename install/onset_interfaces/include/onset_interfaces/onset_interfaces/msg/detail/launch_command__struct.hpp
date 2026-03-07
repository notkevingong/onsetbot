// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from onset_interfaces:msg/LaunchCommand.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__STRUCT_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__onset_interfaces__msg__LaunchCommand __attribute__((deprecated))
#else
# define DEPRECATED__onset_interfaces__msg__LaunchCommand __declspec(deprecated)
#endif

namespace onset_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LaunchCommand_
{
  using Type = LaunchCommand_<ContainerAllocator>;

  explicit LaunchCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0;
      this->angle_launch = 0.0;
      this->angle_turret = 0.0;
      this->home_onset_request = 0;
    }
  }

  explicit LaunchCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0;
      this->angle_launch = 0.0;
      this->angle_turret = 0.0;
      this->home_onset_request = 0;
    }
  }

  // field types and members
  using _velocity_type =
    double;
  _velocity_type velocity;
  using _angle_launch_type =
    double;
  _angle_launch_type angle_launch;
  using _angle_turret_type =
    double;
  _angle_turret_type angle_turret;
  using _home_onset_request_type =
    uint8_t;
  _home_onset_request_type home_onset_request;

  // setters for named parameter idiom
  Type & set__velocity(
    const double & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__angle_launch(
    const double & _arg)
  {
    this->angle_launch = _arg;
    return *this;
  }
  Type & set__angle_turret(
    const double & _arg)
  {
    this->angle_turret = _arg;
    return *this;
  }
  Type & set__home_onset_request(
    const uint8_t & _arg)
  {
    this->home_onset_request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    onset_interfaces::msg::LaunchCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const onset_interfaces::msg::LaunchCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::LaunchCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::LaunchCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__onset_interfaces__msg__LaunchCommand
    std::shared_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__onset_interfaces__msg__LaunchCommand
    std::shared_ptr<onset_interfaces::msg::LaunchCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaunchCommand_ & other) const
  {
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->angle_launch != other.angle_launch) {
      return false;
    }
    if (this->angle_turret != other.angle_turret) {
      return false;
    }
    if (this->home_onset_request != other.home_onset_request) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaunchCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaunchCommand_

// alias to use template instance with default allocator
using LaunchCommand =
  onset_interfaces::msg::LaunchCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__STRUCT_HPP_
