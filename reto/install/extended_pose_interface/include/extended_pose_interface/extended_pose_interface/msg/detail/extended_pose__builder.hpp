// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from extended_pose_interface:msg/ExtendedPose.idl
// generated code does not contain a copyright notice

#ifndef EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__BUILDER_HPP_
#define EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "extended_pose_interface/msg/detail/extended_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace extended_pose_interface
{

namespace msg
{

namespace builder
{

class Init_ExtendedPose_time_stamp
{
public:
  explicit Init_ExtendedPose_time_stamp(::extended_pose_interface::msg::ExtendedPose & msg)
  : msg_(msg)
  {}
  ::extended_pose_interface::msg::ExtendedPose time_stamp(::extended_pose_interface::msg::ExtendedPose::_time_stamp_type arg)
  {
    msg_.time_stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::extended_pose_interface::msg::ExtendedPose msg_;
};

class Init_ExtendedPose_angular_velocity
{
public:
  explicit Init_ExtendedPose_angular_velocity(::extended_pose_interface::msg::ExtendedPose & msg)
  : msg_(msg)
  {}
  Init_ExtendedPose_time_stamp angular_velocity(::extended_pose_interface::msg::ExtendedPose::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_ExtendedPose_time_stamp(msg_);
  }

private:
  ::extended_pose_interface::msg::ExtendedPose msg_;
};

class Init_ExtendedPose_linear_velocity
{
public:
  explicit Init_ExtendedPose_linear_velocity(::extended_pose_interface::msg::ExtendedPose & msg)
  : msg_(msg)
  {}
  Init_ExtendedPose_angular_velocity linear_velocity(::extended_pose_interface::msg::ExtendedPose::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_ExtendedPose_angular_velocity(msg_);
  }

private:
  ::extended_pose_interface::msg::ExtendedPose msg_;
};

class Init_ExtendedPose_pose
{
public:
  Init_ExtendedPose_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExtendedPose_linear_velocity pose(::extended_pose_interface::msg::ExtendedPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ExtendedPose_linear_velocity(msg_);
  }

private:
  ::extended_pose_interface::msg::ExtendedPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::extended_pose_interface::msg::ExtendedPose>()
{
  return extended_pose_interface::msg::builder::Init_ExtendedPose_pose();
}

}  // namespace extended_pose_interface

#endif  // EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__BUILDER_HPP_
