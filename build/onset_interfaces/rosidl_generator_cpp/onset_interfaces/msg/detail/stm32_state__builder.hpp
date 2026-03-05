// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from onset_interfaces:msg/STM32State.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__BUILDER_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "onset_interfaces/msg/detail/stm32_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace onset_interfaces
{

namespace msg
{

namespace builder
{

class Init_STM32State_error_code
{
public:
  explicit Init_STM32State_error_code(::onset_interfaces::msg::STM32State & msg)
  : msg_(msg)
  {}
  ::onset_interfaces::msg::STM32State error_code(::onset_interfaces::msg::STM32State::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::onset_interfaces::msg::STM32State msg_;
};

class Init_STM32State_sw3
{
public:
  explicit Init_STM32State_sw3(::onset_interfaces::msg::STM32State & msg)
  : msg_(msg)
  {}
  Init_STM32State_error_code sw3(::onset_interfaces::msg::STM32State::_sw3_type arg)
  {
    msg_.sw3 = std::move(arg);
    return Init_STM32State_error_code(msg_);
  }

private:
  ::onset_interfaces::msg::STM32State msg_;
};

class Init_STM32State_sw2
{
public:
  explicit Init_STM32State_sw2(::onset_interfaces::msg::STM32State & msg)
  : msg_(msg)
  {}
  Init_STM32State_sw3 sw2(::onset_interfaces::msg::STM32State::_sw2_type arg)
  {
    msg_.sw2 = std::move(arg);
    return Init_STM32State_sw3(msg_);
  }

private:
  ::onset_interfaces::msg::STM32State msg_;
};

class Init_STM32State_sw1
{
public:
  Init_STM32State_sw1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_STM32State_sw2 sw1(::onset_interfaces::msg::STM32State::_sw1_type arg)
  {
    msg_.sw1 = std::move(arg);
    return Init_STM32State_sw2(msg_);
  }

private:
  ::onset_interfaces::msg::STM32State msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::onset_interfaces::msg::STM32State>()
{
  return onset_interfaces::msg::builder::Init_STM32State_sw1();
}

}  // namespace onset_interfaces

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__BUILDER_HPP_
