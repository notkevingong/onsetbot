// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from onset_interfaces:msg/OnsetStatus.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__BUILDER_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "onset_interfaces/msg/detail/onset_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace onset_interfaces
{

namespace msg
{

namespace builder
{

class Init_OnsetStatus_bool_busy
{
public:
  explicit Init_OnsetStatus_bool_busy(::onset_interfaces::msg::OnsetStatus & msg)
  : msg_(msg)
  {}
  ::onset_interfaces::msg::OnsetStatus bool_busy(::onset_interfaces::msg::OnsetStatus::_bool_busy_type arg)
  {
    msg_.bool_busy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::onset_interfaces::msg::OnsetStatus msg_;
};

class Init_OnsetStatus_bool_homed
{
public:
  Init_OnsetStatus_bool_homed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OnsetStatus_bool_busy bool_homed(::onset_interfaces::msg::OnsetStatus::_bool_homed_type arg)
  {
    msg_.bool_homed = std::move(arg);
    return Init_OnsetStatus_bool_busy(msg_);
  }

private:
  ::onset_interfaces::msg::OnsetStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::onset_interfaces::msg::OnsetStatus>()
{
  return onset_interfaces::msg::builder::Init_OnsetStatus_bool_homed();
}

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__BUILDER_HPP_
