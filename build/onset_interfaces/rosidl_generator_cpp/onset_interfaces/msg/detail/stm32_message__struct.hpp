// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from onset_interfaces:msg/STM32Message.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__STRUCT_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__onset_interfaces__msg__STM32Message __attribute__((deprecated))
#else
# define DEPRECATED__onset_interfaces__msg__STM32Message __declspec(deprecated)
#endif

namespace onset_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct STM32Message_
{
  using Type = STM32Message_<ContainerAllocator>;

  explicit STM32Message_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_launch = 0.0;
      this->power_on_status = 0;
      this->home_elbow_request = 0;
    }
  }

  explicit STM32Message_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_launch = 0.0;
      this->power_on_status = 0;
      this->home_elbow_request = 0;
    }
  }

  // field types and members
  using _angle_launch_type =
    double;
  _angle_launch_type angle_launch;
  using _power_on_status_type =
    uint8_t;
  _power_on_status_type power_on_status;
  using _home_elbow_request_type =
    uint8_t;
  _home_elbow_request_type home_elbow_request;

  // setters for named parameter idiom
  Type & set__angle_launch(
    const double & _arg)
  {
    this->angle_launch = _arg;
    return *this;
  }
  Type & set__power_on_status(
    const uint8_t & _arg)
  {
    this->power_on_status = _arg;
    return *this;
  }
  Type & set__home_elbow_request(
    const uint8_t & _arg)
  {
    this->home_elbow_request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    onset_interfaces::msg::STM32Message_<ContainerAllocator> *;
  using ConstRawPtr =
    const onset_interfaces::msg::STM32Message_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::STM32Message_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::STM32Message_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__onset_interfaces__msg__STM32Message
    std::shared_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__onset_interfaces__msg__STM32Message
    std::shared_ptr<onset_interfaces::msg::STM32Message_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const STM32Message_ & other) const
  {
    if (this->angle_launch != other.angle_launch) {
      return false;
    }
    if (this->power_on_status != other.power_on_status) {
      return false;
    }
    if (this->home_elbow_request != other.home_elbow_request) {
      return false;
    }
    return true;
  }
  bool operator!=(const STM32Message_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct STM32Message_

// alias to use template instance with default allocator
using STM32Message =
  onset_interfaces::msg::STM32Message_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__STRUCT_HPP_
