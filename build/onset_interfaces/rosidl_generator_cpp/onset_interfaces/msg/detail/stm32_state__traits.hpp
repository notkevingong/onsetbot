// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from onset_interfaces:msg/STM32State.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__TRAITS_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "onset_interfaces/msg/detail/stm32_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace onset_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const STM32State & msg,
  std::ostream & out)
{
  out << "{";
  // member: sw2
  {
    out << "sw2: ";
    rosidl_generator_traits::value_to_yaml(msg.sw2, out);
    out << ", ";
  }

  // member: sw3
  {
    out << "sw3: ";
    rosidl_generator_traits::value_to_yaml(msg.sw3, out);
    out << ", ";
  }

  // member: elbow_moving_status
  {
    out << "elbow_moving_status: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow_moving_status, out);
    out << ", ";
  }

  // member: elbow_power_status
  {
    out << "elbow_power_status: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow_power_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const STM32State & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sw2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sw2: ";
    rosidl_generator_traits::value_to_yaml(msg.sw2, out);
    out << "\n";
  }

  // member: sw3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sw3: ";
    rosidl_generator_traits::value_to_yaml(msg.sw3, out);
    out << "\n";
  }

  // member: elbow_moving_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elbow_moving_status: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow_moving_status, out);
    out << "\n";
  }

  // member: elbow_power_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elbow_power_status: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow_power_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const STM32State & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace onset_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use onset_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const onset_interfaces::msg::STM32State & msg,
  std::ostream & out, size_t indentation = 0)
{
  onset_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use onset_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const onset_interfaces::msg::STM32State & msg)
{
  return onset_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<onset_interfaces::msg::STM32State>()
{
  return "onset_interfaces::msg::STM32State";
}

template<>
inline const char * name<onset_interfaces::msg::STM32State>()
{
  return "onset_interfaces/msg/STM32State";
}

template<>
struct has_fixed_size<onset_interfaces::msg::STM32State>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<onset_interfaces::msg::STM32State>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<onset_interfaces::msg::STM32State>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__TRAITS_HPP_
