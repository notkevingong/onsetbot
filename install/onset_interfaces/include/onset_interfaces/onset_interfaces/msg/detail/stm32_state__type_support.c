// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from onset_interfaces:msg/STM32State.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "onset_interfaces/msg/detail/stm32_state__rosidl_typesupport_introspection_c.h"
#include "onset_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "onset_interfaces/msg/detail/stm32_state__functions.h"
#include "onset_interfaces/msg/detail/stm32_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  onset_interfaces__msg__STM32State__init(message_memory);
}

void onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_fini_function(void * message_memory)
{
  onset_interfaces__msg__STM32State__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_member_array[4] = {
  {
    "sw1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(onset_interfaces__msg__STM32State, sw1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sw2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(onset_interfaces__msg__STM32State, sw2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sw3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(onset_interfaces__msg__STM32State, sw3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(onset_interfaces__msg__STM32State, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_members = {
  "onset_interfaces__msg",  // message namespace
  "STM32State",  // message name
  4,  // number of fields
  sizeof(onset_interfaces__msg__STM32State),
  onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_member_array,  // message members
  onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_init_function,  // function to initialize message memory (memory has to be allocated)
  onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_type_support_handle = {
  0,
  &onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_onset_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, onset_interfaces, msg, STM32State)() {
  if (!onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_type_support_handle.typesupport_identifier) {
    onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &onset_interfaces__msg__STM32State__rosidl_typesupport_introspection_c__STM32State_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
