// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from onset_interfaces:msg/LaunchCommand.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "onset_interfaces/msg/detail/launch_command__struct.h"
#include "onset_interfaces/msg/detail/launch_command__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace onset_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _LaunchCommand_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LaunchCommand_type_support_ids_t;

static const _LaunchCommand_type_support_ids_t _LaunchCommand_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LaunchCommand_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LaunchCommand_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LaunchCommand_type_support_symbol_names_t _LaunchCommand_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, onset_interfaces, msg, LaunchCommand)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, onset_interfaces, msg, LaunchCommand)),
  }
};

typedef struct _LaunchCommand_type_support_data_t
{
  void * data[2];
} _LaunchCommand_type_support_data_t;

static _LaunchCommand_type_support_data_t _LaunchCommand_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LaunchCommand_message_typesupport_map = {
  2,
  "onset_interfaces",
  &_LaunchCommand_message_typesupport_ids.typesupport_identifier[0],
  &_LaunchCommand_message_typesupport_symbol_names.symbol_name[0],
  &_LaunchCommand_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LaunchCommand_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LaunchCommand_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace onset_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, onset_interfaces, msg, LaunchCommand)() {
  return &::onset_interfaces::msg::rosidl_typesupport_c::LaunchCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
