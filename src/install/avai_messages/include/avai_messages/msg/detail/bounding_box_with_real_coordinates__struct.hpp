// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from avai_messages:msg/BoundingBoxWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__STRUCT_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__avai_messages__msg__BoundingBoxWithRealCoordinates __attribute__((deprecated))
#else
# define DEPRECATED__avai_messages__msg__BoundingBoxWithRealCoordinates __declspec(deprecated)
#endif

namespace avai_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBoxWithRealCoordinates_
{
  using Type = BoundingBoxWithRealCoordinates_<ContainerAllocator>;

  explicit BoundingBoxWithRealCoordinates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->image_coords.begin(), this->image_coords.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->real_coords.begin(), this->real_coords.end(), 0.0f);
      this->conf = 0.0f;
      this->cls = 0.0f;
    }
  }

  explicit BoundingBoxWithRealCoordinates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image_coords(_alloc),
    real_coords(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->image_coords.begin(), this->image_coords.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->real_coords.begin(), this->real_coords.end(), 0.0f);
      this->conf = 0.0f;
      this->cls = 0.0f;
    }
  }

  // field types and members
  using _image_coords_type =
    std::array<float, 4>;
  _image_coords_type image_coords;
  using _real_coords_type =
    std::array<float, 2>;
  _real_coords_type real_coords;
  using _conf_type =
    float;
  _conf_type conf;
  using _cls_type =
    float;
  _cls_type cls;

  // setters for named parameter idiom
  Type & set__image_coords(
    const std::array<float, 4> & _arg)
  {
    this->image_coords = _arg;
    return *this;
  }
  Type & set__real_coords(
    const std::array<float, 2> & _arg)
  {
    this->real_coords = _arg;
    return *this;
  }
  Type & set__conf(
    const float & _arg)
  {
    this->conf = _arg;
    return *this;
  }
  Type & set__cls(
    const float & _arg)
  {
    this->cls = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator> *;
  using ConstRawPtr =
    const avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__avai_messages__msg__BoundingBoxWithRealCoordinates
    std::shared_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__avai_messages__msg__BoundingBoxWithRealCoordinates
    std::shared_ptr<avai_messages::msg::BoundingBoxWithRealCoordinates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBoxWithRealCoordinates_ & other) const
  {
    if (this->image_coords != other.image_coords) {
      return false;
    }
    if (this->real_coords != other.real_coords) {
      return false;
    }
    if (this->conf != other.conf) {
      return false;
    }
    if (this->cls != other.cls) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBoxWithRealCoordinates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBoxWithRealCoordinates_

// alias to use template instance with default allocator
using BoundingBoxWithRealCoordinates =
  avai_messages::msg::BoundingBoxWithRealCoordinates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__STRUCT_HPP_
