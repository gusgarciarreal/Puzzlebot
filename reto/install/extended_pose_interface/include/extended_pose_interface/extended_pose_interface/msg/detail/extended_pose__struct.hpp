// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from extended_pose_interface:msg/ExtendedPose.idl
// generated code does not contain a copyright notice

#ifndef EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__STRUCT_HPP_
#define EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__extended_pose_interface__msg__ExtendedPose __attribute__((deprecated))
#else
# define DEPRECATED__extended_pose_interface__msg__ExtendedPose __declspec(deprecated)
#endif

namespace extended_pose_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ExtendedPose_
{
  using Type = ExtendedPose_<ContainerAllocator>;

  explicit ExtendedPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->time_stamp = 0.0f;
    }
  }

  explicit ExtendedPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->time_stamp = 0.0f;
    }
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _linear_velocity_type =
    float;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    float;
  _angular_velocity_type angular_velocity;
  using _time_stamp_type =
    float;
  _time_stamp_type time_stamp;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const float & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const float & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__time_stamp(
    const float & _arg)
  {
    this->time_stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    extended_pose_interface::msg::ExtendedPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const extended_pose_interface::msg::ExtendedPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      extended_pose_interface::msg::ExtendedPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      extended_pose_interface::msg::ExtendedPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__extended_pose_interface__msg__ExtendedPose
    std::shared_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__extended_pose_interface__msg__ExtendedPose
    std::shared_ptr<extended_pose_interface::msg::ExtendedPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExtendedPose_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->time_stamp != other.time_stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExtendedPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExtendedPose_

// alias to use template instance with default allocator
using ExtendedPose =
  extended_pose_interface::msg::ExtendedPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace extended_pose_interface

#endif  // EXTENDED_POSE_INTERFACE__MSG__DETAIL__EXTENDED_POSE__STRUCT_HPP_
