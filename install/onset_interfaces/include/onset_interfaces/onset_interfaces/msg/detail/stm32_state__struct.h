// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from onset_interfaces:msg/STM32State.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__STRUCT_H_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/STM32State in the package onset_interfaces.
typedef struct onset_interfaces__msg__STM32State
{
  uint8_t sw2;
  uint8_t sw3;
  uint8_t elbow_moving_status;
  uint8_t elbow_power_status;
} onset_interfaces__msg__STM32State;

// Struct for a sequence of onset_interfaces__msg__STM32State.
typedef struct onset_interfaces__msg__STM32State__Sequence
{
  onset_interfaces__msg__STM32State * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} onset_interfaces__msg__STM32State__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__STRUCT_H_
