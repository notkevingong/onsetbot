// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from onset_interfaces:msg/LaunchCommand.idl
// generated code does not contain a copyright notice
#include "onset_interfaces/msg/detail/launch_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
onset_interfaces__msg__LaunchCommand__init(onset_interfaces__msg__LaunchCommand * msg)
{
  if (!msg) {
    return false;
  }
  // velocity
  // angle_launch
  // angle_turret
  // home_seq
  return true;
}

void
onset_interfaces__msg__LaunchCommand__fini(onset_interfaces__msg__LaunchCommand * msg)
{
  if (!msg) {
    return;
  }
  // velocity
  // angle_launch
  // angle_turret
  // home_seq
}

bool
onset_interfaces__msg__LaunchCommand__are_equal(const onset_interfaces__msg__LaunchCommand * lhs, const onset_interfaces__msg__LaunchCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // angle_launch
  if (lhs->angle_launch != rhs->angle_launch) {
    return false;
  }
  // angle_turret
  if (lhs->angle_turret != rhs->angle_turret) {
    return false;
  }
  // home_seq
  if (lhs->home_seq != rhs->home_seq) {
    return false;
  }
  return true;
}

bool
onset_interfaces__msg__LaunchCommand__copy(
  const onset_interfaces__msg__LaunchCommand * input,
  onset_interfaces__msg__LaunchCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // velocity
  output->velocity = input->velocity;
  // angle_launch
  output->angle_launch = input->angle_launch;
  // angle_turret
  output->angle_turret = input->angle_turret;
  // home_seq
  output->home_seq = input->home_seq;
  return true;
}

onset_interfaces__msg__LaunchCommand *
onset_interfaces__msg__LaunchCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  onset_interfaces__msg__LaunchCommand * msg = (onset_interfaces__msg__LaunchCommand *)allocator.allocate(sizeof(onset_interfaces__msg__LaunchCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(onset_interfaces__msg__LaunchCommand));
  bool success = onset_interfaces__msg__LaunchCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
onset_interfaces__msg__LaunchCommand__destroy(onset_interfaces__msg__LaunchCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    onset_interfaces__msg__LaunchCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
onset_interfaces__msg__LaunchCommand__Sequence__init(onset_interfaces__msg__LaunchCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  onset_interfaces__msg__LaunchCommand * data = NULL;

  if (size) {
    data = (onset_interfaces__msg__LaunchCommand *)allocator.zero_allocate(size, sizeof(onset_interfaces__msg__LaunchCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = onset_interfaces__msg__LaunchCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        onset_interfaces__msg__LaunchCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
onset_interfaces__msg__LaunchCommand__Sequence__fini(onset_interfaces__msg__LaunchCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      onset_interfaces__msg__LaunchCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

onset_interfaces__msg__LaunchCommand__Sequence *
onset_interfaces__msg__LaunchCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  onset_interfaces__msg__LaunchCommand__Sequence * array = (onset_interfaces__msg__LaunchCommand__Sequence *)allocator.allocate(sizeof(onset_interfaces__msg__LaunchCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = onset_interfaces__msg__LaunchCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
onset_interfaces__msg__LaunchCommand__Sequence__destroy(onset_interfaces__msg__LaunchCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    onset_interfaces__msg__LaunchCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
onset_interfaces__msg__LaunchCommand__Sequence__are_equal(const onset_interfaces__msg__LaunchCommand__Sequence * lhs, const onset_interfaces__msg__LaunchCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!onset_interfaces__msg__LaunchCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
onset_interfaces__msg__LaunchCommand__Sequence__copy(
  const onset_interfaces__msg__LaunchCommand__Sequence * input,
  onset_interfaces__msg__LaunchCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(onset_interfaces__msg__LaunchCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    onset_interfaces__msg__LaunchCommand * data =
      (onset_interfaces__msg__LaunchCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!onset_interfaces__msg__LaunchCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          onset_interfaces__msg__LaunchCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!onset_interfaces__msg__LaunchCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
