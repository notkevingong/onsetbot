// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from onset_interfaces:msg/LaunchCommand.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__STRUCT_H_
#define ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LaunchCommand in the package onset_interfaces.
typedef struct onset_interfaces__msg__LaunchCommand
{
  double velocity;
  double angle_launch;
  double angle_turret;
  uint8_t home_onset_request;
} onset_interfaces__msg__LaunchCommand;

// Struct for a sequence of onset_interfaces__msg__LaunchCommand.
typedef struct onset_interfaces__msg__LaunchCommand__Sequence
{
  onset_interfaces__msg__LaunchCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} onset_interfaces__msg__LaunchCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ONSET_INTERFACES__MSG__DETAIL__LAUNCH_COMMAND__STRUCT_H_
