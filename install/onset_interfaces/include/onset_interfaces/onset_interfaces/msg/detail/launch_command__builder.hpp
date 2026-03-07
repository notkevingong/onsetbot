// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from onset_interfaces:msg/LaunchCommand.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__BUILDER_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "onset_interfaces/msg/detail/launch_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace onset_interfaces
{

namespace msg
{

namespace builder
{

class Init_LaunchCommand_home_onset_request
{
public:
  explicit Init_LaunchCommand_home_onset_request(::onset_interfaces::msg::LaunchCommand & msg)
  : msg_(msg)
  {}
  ::onset_interfaces::msg::LaunchCommand home_onset_request(::onset_interfaces::msg::LaunchCommand::_home_onset_request_type arg)
  {
    msg_.home_onset_request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::onset_interfaces::msg::LaunchCommand msg_;
};

class Init_LaunchCommand_angle_turret
{
public:
  explicit Init_LaunchCommand_angle_turret(::onset_interfaces::msg::LaunchCommand & msg)
  : msg_(msg)
  {}
  Init_LaunchCommand_home_onset_request angle_turret(::onset_interfaces::msg::LaunchCommand::_angle_turret_type arg)
  {
    msg_.angle_turret = std::move(arg);
    return Init_LaunchCommand_home_onset_request(msg_);
  }

private:
  ::onset_interfaces::msg::LaunchCommand msg_;
};

class Init_LaunchCommand_angle_launch
{
public:
  explicit Init_LaunchCommand_angle_launch(::onset_interfaces::msg::LaunchCommand & msg)
  : msg_(msg)
  {}
  Init_LaunchCommand_angle_turret angle_launch(::onset_interfaces::msg::LaunchCommand::_angle_launch_type arg)
  {
    msg_.angle_launch = std::move(arg);
    return Init_LaunchCommand_angle_turret(msg_);
  }

private:
  ::onset_interfaces::msg::LaunchCommand msg_;
};

class Init_LaunchCommand_velocity
{
public:
  Init_LaunchCommand_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaunchCommand_angle_launch velocity(::onset_interfaces::msg::LaunchCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_LaunchCommand_angle_launch(msg_);
  }

private:
  ::onset_interfaces::msg::LaunchCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::onset_interfaces::msg::LaunchCommand>()
{
  return onset_interfaces::msg::builder::Init_LaunchCommand_velocity();
}

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__BUILDER_HPP_
