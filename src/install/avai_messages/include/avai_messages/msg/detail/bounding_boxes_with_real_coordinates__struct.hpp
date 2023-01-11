// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from avai_messages:msg/BoundingBoxesWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__STRUCT_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'bboxes'
#include "avai_messages/msg/detail/bounding_box_with_real_coordinates__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__avai_messages__msg__BoundingBoxesWithRealCoordinates __attribute__((deprecated))
#else
# define DEPRECATED__avai_messages__msg__BoundingBoxesWithRealCoordinates __declspec(deprecated)
#endif

namespace avai_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBoxesWithRealCoordinates_
{
  using Type = BoundingBoxesWithRealCoordinates_<ContainerAllocator>;

  explicit BoundingBoxesWithRealCoordinates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BoundingBoxesWithRealCoordinates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _bboxes_type =
    std::vector<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>, typename ContainerAllocator::template rebind<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>>::other>;
  _bboxes_type bboxes;

  // setters for named parameter idiom
  Type & set__bboxes(
    const std::vector<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>, typename ContainerAllocator::template rebind<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>>::other> & _arg)
  {
    this->bboxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator> *;
  using ConstRawPtr =
    const avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__avai_messages__msg__BoundingBoxesWithRealCoordinates
    std::shared_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__avai_messages__msg__BoundingBoxesWithRealCoordinates
    std::shared_ptr<avai_messages::msg::BoundingBoxesWithRealCoordinates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBoxesWithRealCoordinates_ & other) const
  {
    if (this->bboxes != other.bboxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBoxesWithRealCoordinates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBoxesWithRealCoordinates_

// alias to use template instance with default allocator
using BoundingBoxesWithRealCoordinates =
  avai_messages::msg::BoundingBoxesWithRealCoordinates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__STRUCT_HPP_
