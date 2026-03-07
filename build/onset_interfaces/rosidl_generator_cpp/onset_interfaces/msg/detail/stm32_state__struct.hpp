// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from onset_interfaces:msg/STM32State.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__STRUCT_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__onset_interfaces__msg__STM32State __attribute__((deprecated))
#else
# define DEPRECATED__onset_interfaces__msg__STM32State __declspec(deprecated)
#endif

namespace onset_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct STM32State_
{
  using Type = STM32State_<ContainerAllocator>;

  explicit STM32State_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sw2 = 0;
      this->sw3 = 0;
      this->elbow_moving_status = 0;
      this->elbow_power_status = 0;
    }
  }

  explicit STM32State_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sw2 = 0;
      this->sw3 = 0;
      this->elbow_moving_status = 0;
      this->elbow_power_status = 0;
    }
  }

  // field types and members
  using _sw2_type =
    uint8_t;
  _sw2_type sw2;
  using _sw3_type =
    uint8_t;
  _sw3_type sw3;
  using _elbow_moving_status_type =
    uint8_t;
  _elbow_moving_status_type elbow_moving_status;
  using _elbow_power_status_type =
    uint8_t;
  _elbow_power_status_type elbow_power_status;

  // setters for named parameter idiom
  Type & set__sw2(
    const uint8_t & _arg)
  {
    this->sw2 = _arg;
    return *this;
  }
  Type & set__sw3(
    const uint8_t & _arg)
  {
    this->sw3 = _arg;
    return *this;
  }
  Type & set__elbow_moving_status(
    const uint8_t & _arg)
  {
    this->elbow_moving_status = _arg;
    return *this;
  }
  Type & set__elbow_power_status(
    const uint8_t & _arg)
  {
    this->elbow_power_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    onset_interfaces::msg::STM32State_<ContainerAllocator> *;
  using ConstRawPtr =
    const onset_interfaces::msg::STM32State_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::STM32State_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::STM32State_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__onset_interfaces__msg__STM32State
    std::shared_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__onset_interfaces__msg__STM32State
    std::shared_ptr<onset_interfaces::msg::STM32State_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const STM32State_ & other) const
  {
    if (this->sw2 != other.sw2) {
      return false;
    }
    if (this->sw3 != other.sw3) {
      return false;
    }
    if (this->elbow_moving_status != other.elbow_moving_status) {
      return false;
    }
    if (this->elbow_power_status != other.elbow_power_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const STM32State_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct STM32State_

// alias to use template instance with default allocator
using STM32State =
  onset_interfaces::msg::STM32State_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__STRUCT_HPP_
