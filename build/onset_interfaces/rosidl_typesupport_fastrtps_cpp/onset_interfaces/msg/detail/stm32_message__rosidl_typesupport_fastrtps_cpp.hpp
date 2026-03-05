// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from onset_interfaces:msg/STM32Message.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "onset_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "onset_interfaces/msg/detail/stm32_message__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace onset_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_onset_interfaces
cdr_serialize(
  const onset_interfaces::msg::STM32Message & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_onset_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  onset_interfaces::msg::STM32Message & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_onset_interfaces
get_serialized_size(
  const onset_interfaces::msg::STM32Message & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_onset_interfaces
max_serialized_size_STM32Message(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace onset_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_onset_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, onset_interfaces, msg, STM32Message)();

#ifdef __cplusplus
}
#endif

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
