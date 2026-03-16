// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from onset_interfaces:msg/LaunchCommand.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__TRAITS_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "onset_interfaces/msg/detail/launch_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace onset_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const LaunchCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: angle_launch
  {
    out << "angle_launch: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_launch, out);
    out << ", ";
  }

  // member: angle_turret
  {
    out << "angle_turret: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_turret, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaunchCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: angle_launch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_launch: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_launch, out);
    out << "\n";
  }

  // member: angle_turret
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_turret: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_turret, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaunchCommand & msg, bool use_flow_style = false)
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
  const onset_interfaces::msg::LaunchCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  onset_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use onset_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const onset_interfaces::msg::LaunchCommand & msg)
{
  return onset_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<onset_interfaces::msg::LaunchCommand>()
{
  return "onset_interfaces::msg::LaunchCommand";
}

template<>
inline const char * name<onset_interfaces::msg::LaunchCommand>()
{
  return "onset_interfaces/msg/LaunchCommand";
}

template<>
struct has_fixed_size<onset_interfaces::msg::LaunchCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<onset_interfaces::msg::LaunchCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<onset_interfaces::msg::LaunchCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__TRAITS_HPP_
