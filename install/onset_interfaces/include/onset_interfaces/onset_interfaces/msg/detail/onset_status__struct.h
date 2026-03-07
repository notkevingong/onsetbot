// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from onset_interfaces:msg/OnsetStatus.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__STRUCT_H_
#define ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/OnsetStatus in the package onset_interfaces.
typedef struct onset_interfaces__msg__OnsetStatus
{
  uint8_t onset_is_homed;
  uint8_t onset_is_busy;
} onset_interfaces__msg__OnsetStatus;

// Struct for a sequence of onset_interfaces__msg__OnsetStatus.
typedef struct onset_interfaces__msg__OnsetStatus__Sequence
{
  onset_interfaces__msg__OnsetStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} onset_interfaces__msg__OnsetStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ONSET_INTERFACES__MSG__DETAIL__ONSET_STATUS__STRUCT_H_
