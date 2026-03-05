// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from onset_interfaces:msg/OnsetStatus.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__STRUCT_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__onset_interfaces__msg__OnsetStatus __attribute__((deprecated))
#else
# define DEPRECATED__onset_interfaces__msg__OnsetStatus __declspec(deprecated)
#endif

namespace onset_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OnsetStatus_
{
  using Type = OnsetStatus_<ContainerAllocator>;

  explicit OnsetStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bool_homed = false;
      this->bool_busy = false;
    }
  }

  explicit OnsetStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bool_homed = false;
      this->bool_busy = false;
    }
  }

  // field types and members
  using _bool_homed_type =
    bool;
  _bool_homed_type bool_homed;
  using _bool_busy_type =
    bool;
  _bool_busy_type bool_busy;

  // setters for named parameter idiom
  Type & set__bool_homed(
    const bool & _arg)
  {
    this->bool_homed = _arg;
    return *this;
  }
  Type & set__bool_busy(
    const bool & _arg)
  {
    this->bool_busy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    onset_interfaces::msg::OnsetStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const onset_interfaces::msg::OnsetStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::OnsetStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      onset_interfaces::msg::OnsetStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__onset_interfaces__msg__OnsetStatus
    std::shared_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__onset_interfaces__msg__OnsetStatus
    std::shared_ptr<onset_interfaces::msg::OnsetStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OnsetStatus_ & other) const
  {
    if (this->bool_homed != other.bool_homed) {
      return false;
    }
    if (this->bool_busy != other.bool_busy) {
      return false;
    }
    return true;
  }
  bool operator!=(const OnsetStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OnsetStatus_

// alias to use template instance with default allocator
using OnsetStatus =
  onset_interfaces::msg::OnsetStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__STRUCT_HPP_
