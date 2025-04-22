// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from extended_pose_interface:msg/ExtendedPose.idl
// generated code does not contain a copyright notice

#ifndef EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__STRUCT_H_
#define EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/ExtendedPose in the package extended_pose_interface.
typedef struct extended_pose_interface__msg__ExtendedPose
{
  geometry_msgs__msg__Pose pose;
  float linear_velocity;
  float angular_velocity;
  float time_stamp;
} extended_pose_interface__msg__ExtendedPose;

// Struct for a sequence of extended_pose_interface__msg__ExtendedPose.
typedef struct extended_pose_interface__msg__ExtendedPose__Sequence
{
  extended_pose_interface__msg__ExtendedPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} extended_pose_interface__msg__ExtendedPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__STRUCT_H_
