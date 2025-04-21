// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from extended_pose_interface:msg/ExtendedPose.idl
// generated code does not contain a copyright notice

#ifndef EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__FUNCTIONS_H_
#define EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "extended_pose_interface/msg/rosidl_generator_c__visibility_control.h"

#include "extended_pose_interface/msg/detail/extended_pose__struct.h"

/// Initialize msg/ExtendedPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * extended_pose_interface__msg__ExtendedPose
 * )) before or use
 * extended_pose_interface__msg__ExtendedPose__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
bool
extended_pose_interface__msg__ExtendedPose__init(extended_pose_interface__msg__ExtendedPose * msg);

/// Finalize msg/ExtendedPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
void
extended_pose_interface__msg__ExtendedPose__fini(extended_pose_interface__msg__ExtendedPose * msg);

/// Create msg/ExtendedPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * extended_pose_interface__msg__ExtendedPose__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
extended_pose_interface__msg__ExtendedPose *
extended_pose_interface__msg__ExtendedPose__create();

/// Destroy msg/ExtendedPose message.
/**
 * It calls
 * extended_pose_interface__msg__ExtendedPose__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
void
extended_pose_interface__msg__ExtendedPose__destroy(extended_pose_interface__msg__ExtendedPose * msg);

/// Check for msg/ExtendedPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
bool
extended_pose_interface__msg__ExtendedPose__are_equal(const extended_pose_interface__msg__ExtendedPose * lhs, const extended_pose_interface__msg__ExtendedPose * rhs);

/// Copy a msg/ExtendedPose message.
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
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
bool
extended_pose_interface__msg__ExtendedPose__copy(
  const extended_pose_interface__msg__ExtendedPose * input,
  extended_pose_interface__msg__ExtendedPose * output);

/// Initialize array of msg/ExtendedPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * extended_pose_interface__msg__ExtendedPose__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
bool
extended_pose_interface__msg__ExtendedPose__Sequence__init(extended_pose_interface__msg__ExtendedPose__Sequence * array, size_t size);

/// Finalize array of msg/ExtendedPose messages.
/**
 * It calls
 * extended_pose_interface__msg__ExtendedPose__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
void
extended_pose_interface__msg__ExtendedPose__Sequence__fini(extended_pose_interface__msg__ExtendedPose__Sequence * array);

/// Create array of msg/ExtendedPose messages.
/**
 * It allocates the memory for the array and calls
 * extended_pose_interface__msg__ExtendedPose__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
extended_pose_interface__msg__ExtendedPose__Sequence *
extended_pose_interface__msg__ExtendedPose__Sequence__create(size_t size);

/// Destroy array of msg/ExtendedPose messages.
/**
 * It calls
 * extended_pose_interface__msg__ExtendedPose__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
void
extended_pose_interface__msg__ExtendedPose__Sequence__destroy(extended_pose_interface__msg__ExtendedPose__Sequence * array);

/// Check for msg/ExtendedPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
bool
extended_pose_interface__msg__ExtendedPose__Sequence__are_equal(const extended_pose_interface__msg__ExtendedPose__Sequence * lhs, const extended_pose_interface__msg__ExtendedPose__Sequence * rhs);

/// Copy an array of msg/ExtendedPose messages.
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
ROSIDL_GENERATOR_C_PUBLIC_extended_pose_interface
bool
extended_pose_interface__msg__ExtendedPose__Sequence__copy(
  const extended_pose_interface__msg__ExtendedPose__Sequence * input,
  extended_pose_interface__msg__ExtendedPose__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__FUNCTIONS_H_
