// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from onset_interfaces:msg/STM32Message.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__BUILDER_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "onset_interfaces/msg/detail/stm32_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace onset_interfaces
{

namespace msg
{

namespace builder
{

class Init_STM32Message_home_elbow_request
{
public:
  explicit Init_STM32Message_home_elbow_request(::onset_interfaces::msg::STM32Message & msg)
  : msg_(msg)
  {}
  ::onset_interfaces::msg::STM32Message home_elbow_request(::onset_interfaces::msg::STM32Message::_home_elbow_request_type arg)
  {
    msg_.home_elbow_request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::onset_interfaces::msg::STM32Message msg_;
};

class Init_STM32Message_power_on_status
{
public:
  explicit Init_STM32Message_power_on_status(::onset_interfaces::msg::STM32Message & msg)
  : msg_(msg)
  {}
  Init_STM32Message_home_elbow_request power_on_status(::onset_interfaces::msg::STM32Message::_power_on_status_type arg)
  {
    msg_.power_on_status = std::move(arg);
    return Init_STM32Message_home_elbow_request(msg_);
  }

private:
  ::onset_interfaces::msg::STM32Message msg_;
};

class Init_STM32Message_angle_launch
{
public:
  Init_STM32Message_angle_launch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_STM32Message_power_on_status angle_launch(::onset_interfaces::msg::STM32Message::_angle_launch_type arg)
  {
    msg_.angle_launch = std::move(arg);
    return Init_STM32Message_power_on_status(msg_);
  }

private:
  ::onset_interfaces::msg::STM32Message msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::onset_interfaces::msg::STM32Message>()
{
  return onset_interfaces::msg::builder::Init_STM32Message_angle_launch();
}

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__BUILDER_HPP_
