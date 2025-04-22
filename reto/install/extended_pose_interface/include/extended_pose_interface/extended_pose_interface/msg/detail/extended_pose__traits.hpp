// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from extended_pose_interface:msg/ExtendedPose.idl
// generated code does not contain a copyright notice

#ifndef EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__TRAITS_HPP_
#define EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "extended_pose_interface/msg/detail/extended_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace extended_pose_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const ExtendedPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: linear_velocity
  {
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: time_stamp
  {
    out << "time_stamp: ";
    rosidl_generator_traits::value_to_yaml(msg.time_stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExtendedPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << "\n";
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << "\n";
  }

  // member: time_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_stamp: ";
    rosidl_generator_traits::value_to_yaml(msg.time_stamp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExtendedPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace extended_pose_interface

namespace rosidl_generator_traits
{

[[deprecated("use extended_pose_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const extended_pose_interface::msg::ExtendedPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  extended_pose_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use extended_pose_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const extended_pose_interface::msg::ExtendedPose & msg)
{
  return extended_pose_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<extended_pose_interface::msg::ExtendedPose>()
{
  return "extended_pose_interface::msg::ExtendedPose";
}

template<>
inline const char * name<extended_pose_interface::msg::ExtendedPose>()
{
  return "extended_pose_interface/msg/ExtendedPose";
}

template<>
struct has_fixed_size<extended_pose_interface::msg::ExtendedPose>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<extended_pose_interface::msg::ExtendedPose>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<extended_pose_interface::msg::ExtendedPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__TRAITS_HPP_
