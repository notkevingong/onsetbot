// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from onset_interfaces:msg/STM32State.idl
// generated code does not contain a copyright notice

#ifndef ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__FUNCTIONS_H_
#define ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "onset_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "onset_interfaces/msg/detail/stm32_state__struct.h"

/// Initialize msg/STM32State message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * onset_interfaces__msg__STM32State
 * )) before or use
 * onset_interfaces__msg__STM32State__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
bool
onset_interfaces__msg__STM32State__init(onset_interfaces__msg__STM32State * msg);

/// Finalize msg/STM32State message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
void
onset_interfaces__msg__STM32State__fini(onset_interfaces__msg__STM32State * msg);

/// Create msg/STM32State message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * onset_interfaces__msg__STM32State__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
onset_interfaces__msg__STM32State *
onset_interfaces__msg__STM32State__create();

/// Destroy msg/STM32State message.
/**
 * It calls
 * onset_interfaces__msg__STM32State__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
void
onset_interfaces__msg__STM32State__destroy(onset_interfaces__msg__STM32State * msg);

/// Check for msg/STM32State message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
bool
onset_interfaces__msg__STM32State__are_equal(const onset_interfaces__msg__STM32State * lhs, const onset_interfaces__msg__STM32State * rhs);

/// Copy a msg/STM32State message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
bool
onset_interfaces__msg__STM32State__copy(
  const onset_interfaces__msg__STM32State * input,
  onset_interfaces__msg__STM32State * output);

/// Initialize array of msg/STM32State messages.
/**
 * It allocates the memory for the number of elements and calls
 * onset_interfaces__msg__STM32State__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
bool
onset_interfaces__msg__STM32State__Sequence__init(onset_interfaces__msg__STM32State__Sequence * array, size_t size);

/// Finalize array of msg/STM32State messages.
/**
 * It calls
 * onset_interfaces__msg__STM32State__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
void
onset_interfaces__msg__STM32State__Sequence__fini(onset_interfaces__msg__STM32State__Sequence * array);

/// Create array of msg/STM32State messages.
/**
 * It allocates the memory for the array and calls
 * onset_interfaces__msg__STM32State__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
onset_interfaces__msg__STM32State__Sequence *
onset_interfaces__msg__STM32State__Sequence__create(size_t size);

/// Destroy array of msg/STM32State messages.
/**
 * It calls
 * onset_interfaces__msg__STM32State__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
void
onset_interfaces__msg__STM32State__Sequence__destroy(onset_interfaces__msg__STM32State__Sequence * array);

/// Check for msg/STM32State message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
bool
onset_interfaces__msg__STM32State__Sequence__are_equal(const onset_interfaces__msg__STM32State__Sequence * lhs, const onset_interfaces__msg__STM32State__Sequence * rhs);

/// Copy an array of msg/STM32State messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_onset_interfaces
bool
onset_interfaces__msg__STM32State__Sequence__copy(
  const onset_interfaces__msg__STM32State__Sequence * input,
  onset_interfaces__msg__STM32State__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ONSET_INTERFACES__MSG__DETAIL__STM32_STATE__FUNCTIONS_H_
