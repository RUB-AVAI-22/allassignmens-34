// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from avai_messages:msg/MapEntry.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__STRUCT_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__avai_messages__msg__MapEntry __attribute__((deprecated))
#else
# define DEPRECATED__avai_messages__msg__MapEntry __declspec(deprecated)
#endif

namespace avai_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapEntry_
{
  using Type = MapEntry_<ContainerAllocator>;

  explicit MapEntry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->coordinates.begin(), this->coordinates.end(), 0.0f);
      this->cls = 0;
    }
  }

  explicit MapEntry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : coordinates(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->coordinates.begin(), this->coordinates.end(), 0.0f);
      this->cls = 0;
    }
  }

  // field types and members
  using _coordinates_type =
    std::array<float, 2>;
  _coordinates_type coordinates;
  using _cls_type =
    int8_t;
  _cls_type cls;

  // setters for named parameter idiom
  Type & set__coordinates(
    const std::array<float, 2> & _arg)
  {
    this->coordinates = _arg;
    return *this;
  }
  Type & set__cls(
    const int8_t & _arg)
  {
    this->cls = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    avai_messages::msg::MapEntry_<ContainerAllocator> *;
  using ConstRawPtr =
    const avai_messages::msg::MapEntry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<avai_messages::msg::MapEntry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<avai_messages::msg::MapEntry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::MapEntry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::MapEntry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::MapEntry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::MapEntry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<avai_messages::msg::MapEntry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<avai_messages::msg::MapEntry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__avai_messages__msg__MapEntry
    std::shared_ptr<avai_messages::msg::MapEntry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__avai_messages__msg__MapEntry
    std::shared_ptr<avai_messages::msg::MapEntry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapEntry_ & other) const
  {
    if (this->coordinates != other.coordinates) {
      return false;
    }
    if (this->cls != other.cls) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapEntry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapEntry_

// alias to use template instance with default allocator
using MapEntry =
  avai_messages::msg::MapEntry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__STRUCT_HPP_
