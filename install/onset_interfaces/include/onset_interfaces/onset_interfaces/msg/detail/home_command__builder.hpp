// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from onset_interfaces:msg/HomeCommand.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__HOME_COMMAND__BUILDER_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__HOME_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "onset_interfaces/msg/detail/home_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace onset_interfaces
{

namespace msg
{

namespace builder
{

class Init_HomeCommand_home_onset_request
{
public:
  Init_HomeCommand_home_onset_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::onset_interfaces::msg::HomeCommand home_onset_request(::onset_interfaces::msg::HomeCommand::_home_onset_request_type arg)
  {
    msg_.home_onset_request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::onset_interfaces::msg::HomeCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::onset_interfaces::msg::HomeCommand>()
{
  return onset_interfaces::msg::builder::Init_HomeCommand_home_onset_request();
}

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__HOME_COMMAND__BUILDER_HPP_
