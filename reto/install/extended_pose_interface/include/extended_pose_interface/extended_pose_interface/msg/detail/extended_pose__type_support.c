// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from extended_pose_interface:msg/ExtendedPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "extended_pose_interface/msg/detail/extended_pose__rosidl_typesupport_introspection_c.h"
#include "extended_pose_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "extended_pose_interface/msg/detail/extended_pose__functions.h"
#include "extended_pose_interface/msg/detail/extended_pose__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  extended_pose_interface__msg__ExtendedPose__init(message_memory);
}

void extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_fini_function(void * message_memory)
{
  extended_pose_interface__msg__ExtendedPose__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_member_array[4] = {
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(extended_pose_interface__msg__ExtendedPose, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(extended_pose_interface__msg__ExtendedPose, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(extended_pose_interface__msg__ExtendedPose, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(extended_pose_interface__msg__ExtendedPose, time_stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_members = {
  "extended_pose_interface__msg",  // message namespace
  "ExtendedPose",  // message name
  4,  // number of fields
  sizeof(extended_pose_interface__msg__ExtendedPose),
  extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_member_array,  // message members
  extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_init_function,  // function to initialize message memory (memory has to be allocated)
  extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_type_support_handle = {
  0,
  &extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_extended_pose_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, extended_pose_interface, msg, ExtendedPose)() {
  extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_type_support_handle.typesupport_identifier) {
    extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &extended_pose_interface__msg__ExtendedPose__rosidl_typesupport_introspection_c__ExtendedPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
