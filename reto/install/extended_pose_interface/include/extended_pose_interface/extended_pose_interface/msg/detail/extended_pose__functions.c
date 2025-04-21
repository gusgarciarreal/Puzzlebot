// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from extended_pose_interface:msg/ExtendedPose.idl
// generated code does not contain a copyright notice
#include "extended_pose_interface/msg/detail/extended_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
extended_pose_interface__msg__ExtendedPose__init(extended_pose_interface__msg__ExtendedPose * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    extended_pose_interface__msg__ExtendedPose__fini(msg);
    return false;
  }
  // linear_velocity
  // angular_velocity
  // time_stamp
  return true;
}

void
extended_pose_interface__msg__ExtendedPose__fini(extended_pose_interface__msg__ExtendedPose * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // linear_velocity
  // angular_velocity
  // time_stamp
}

bool
extended_pose_interface__msg__ExtendedPose__are_equal(const extended_pose_interface__msg__ExtendedPose * lhs, const extended_pose_interface__msg__ExtendedPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // time_stamp
  if (lhs->time_stamp != rhs->time_stamp) {
    return false;
  }
  return true;
}

bool
extended_pose_interface__msg__ExtendedPose__copy(
  const extended_pose_interface__msg__ExtendedPose * input,
  extended_pose_interface__msg__ExtendedPose * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // time_stamp
  output->time_stamp = input->time_stamp;
  return true;
}

extended_pose_interface__msg__ExtendedPose *
extended_pose_interface__msg__ExtendedPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  extended_pose_interface__msg__ExtendedPose * msg = (extended_pose_interface__msg__ExtendedPose *)allocator.allocate(sizeof(extended_pose_interface__msg__ExtendedPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(extended_pose_interface__msg__ExtendedPose));
  bool success = extended_pose_interface__msg__ExtendedPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
extended_pose_interface__msg__ExtendedPose__destroy(extended_pose_interface__msg__ExtendedPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    extended_pose_interface__msg__ExtendedPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
extended_pose_interface__msg__ExtendedPose__Sequence__init(extended_pose_interface__msg__ExtendedPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  extended_pose_interface__msg__ExtendedPose * data = NULL;

  if (size) {
    data = (extended_pose_interface__msg__ExtendedPose *)allocator.zero_allocate(size, sizeof(extended_pose_interface__msg__ExtendedPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = extended_pose_interface__msg__ExtendedPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        extended_pose_interface__msg__ExtendedPose__fini(&data[i - 1]);
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
extended_pose_interface__msg__ExtendedPose__Sequence__fini(extended_pose_interface__msg__ExtendedPose__Sequence * array)
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
      extended_pose_interface__msg__ExtendedPose__fini(&array->data[i]);
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

extended_pose_interface__msg__ExtendedPose__Sequence *
extended_pose_interface__msg__ExtendedPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  extended_pose_interface__msg__ExtendedPose__Sequence * array = (extended_pose_interface__msg__ExtendedPose__Sequence *)allocator.allocate(sizeof(extended_pose_interface__msg__ExtendedPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = extended_pose_interface__msg__ExtendedPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
extended_pose_interface__msg__ExtendedPose__Sequence__destroy(extended_pose_interface__msg__ExtendedPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    extended_pose_interface__msg__ExtendedPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
extended_pose_interface__msg__ExtendedPose__Sequence__are_equal(const extended_pose_interface__msg__ExtendedPose__Sequence * lhs, const extended_pose_interface__msg__ExtendedPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!extended_pose_interface__msg__ExtendedPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
extended_pose_interface__msg__ExtendedPose__Sequence__copy(
  const extended_pose_interface__msg__ExtendedPose__Sequence * input,
  extended_pose_interface__msg__ExtendedPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(extended_pose_interface__msg__ExtendedPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    extended_pose_interface__msg__ExtendedPose * data =
      (extended_pose_interface__msg__ExtendedPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!extended_pose_interface__msg__ExtendedPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          extended_pose_interface__msg__ExtendedPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!extended_pose_interface__msg__ExtendedPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
