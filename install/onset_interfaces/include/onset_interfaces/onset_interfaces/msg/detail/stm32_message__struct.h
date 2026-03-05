// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from onset_interfaces:msg/STM32Message.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__STRUCT_H_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/STM32Message in the package onset_interfaces.
typedef struct onset_interfaces__msg__STM32Message
{
  double angle_launch;
  uint8_t power_on_status;
  uint8_t home_elbow_request;
} onset_interfaces__msg__STM32Message;

// Struct for a sequence of onset_interfaces__msg__STM32Message.
typedef struct onset_interfaces__msg__STM32Message__Sequence
{
  onset_interfaces__msg__STM32Message * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} onset_interfaces__msg__STM32Message__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_MESSAGE__STRUCT_H_
