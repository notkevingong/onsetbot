// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from onset_interfaces:msg/STM32Message.idl
// generated code does not contain a copyright notice
#include "onset_interfaces/msg/detail/stm32_message__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "onset_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "onset_interfaces/msg/detail/stm32_message__struct.h"
#include "onset_interfaces/msg/detail/stm32_message__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _STM32Message__ros_msg_type = onset_interfaces__msg__STM32Message;

static bool _STM32Message__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _STM32Message__ros_msg_type * ros_message = static_cast<const _STM32Message__ros_msg_type *>(untyped_ros_message);
  // Field name: angle_launch
  {
    cdr << ros_message->angle_launch;
  }

  // Field name: power_on_status
  {
    cdr << ros_message->power_on_status;
  }

  // Field name: home_elbow_request
  {
    cdr << ros_message->home_elbow_request;
  }

  return true;
}

static bool _STM32Message__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _STM32Message__ros_msg_type * ros_message = static_cast<_STM32Message__ros_msg_type *>(untyped_ros_message);
  // Field name: angle_launch
  {
    cdr >> ros_message->angle_launch;
  }

  // Field name: power_on_status
  {
    cdr >> ros_message->power_on_status;
  }

  // Field name: home_elbow_request
  {
    cdr >> ros_message->home_elbow_request;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_onset_interfaces
size_t get_serialized_size_onset_interfaces__msg__STM32Message(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _STM32Message__ros_msg_type * ros_message = static_cast<const _STM32Message__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name angle_launch
  {
    size_t item_size = sizeof(ros_message->angle_launch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name power_on_status
  {
    size_t item_size = sizeof(ros_message->power_on_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name home_elbow_request
  {
    size_t item_size = sizeof(ros_message->home_elbow_request);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _STM32Message__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_onset_interfaces__msg__STM32Message(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_onset_interfaces
size_t max_serialized_size_onset_interfaces__msg__STM32Message(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: angle_launch
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: power_on_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: home_elbow_request
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = onset_interfaces__msg__STM32Message;
    is_plain =
      (
      offsetof(DataType, home_elbow_request) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _STM32Message__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_onset_interfaces__msg__STM32Message(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_STM32Message = {
  "onset_interfaces::msg",
  "STM32Message",
  _STM32Message__cdr_serialize,
  _STM32Message__cdr_deserialize,
  _STM32Message__get_serialized_size,
  _STM32Message__max_serialized_size
};

static rosidl_message_type_support_t _STM32Message__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_STM32Message,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, onset_interfaces, msg, STM32Message)() {
  return &_STM32Message__type_support;
}

#if defined(__cplusplus)
}
#endif
