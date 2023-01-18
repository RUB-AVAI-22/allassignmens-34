// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from avai_messages:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__MAP__STRUCT_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__MAP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'map_objects'
#include "avai_messages/msg/detail/map_entry__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__avai_messages__msg__Map __attribute__((deprecated))
#else
# define DEPRECATED__avai_messages__msg__Map __declspec(deprecated)
#endif

namespace avai_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Map_
{
  using Type = Map_<ContainerAllocator>;

  explicit Map_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Map_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _map_objects_type =
    std::vector<avai_messages::msg::MapEntry_<ContainerAllocator>, typename ContainerAllocator::template rebind<avai_messages::msg::MapEntry_<ContainerAllocator>>::other>;
  _map_objects_type map_objects;

  // setters for named parameter idiom
  Type & set__map_objects(
    const std::vector<avai_messages::msg::MapEntry_<ContainerAllocator>, typename ContainerAllocator::template rebind<avai_messages::msg::MapEntry_<ContainerAllocator>>::other> & _arg)
  {
    this->map_objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    avai_messages::msg::Map_<ContainerAllocator> *;
  using ConstRawPtr =
    const avai_messages::msg::Map_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<avai_messages::msg::Map_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<avai_messages::msg::Map_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::Map_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::Map_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      avai_messages::msg::Map_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<avai_messages::msg::Map_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<avai_messages::msg::Map_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<avai_messages::msg::Map_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__avai_messages__msg__Map
    std::shared_ptr<avai_messages::msg::Map_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__avai_messages__msg__Map
    std::shared_ptr<avai_messages::msg::Map_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Map_ & other) const
  {
    if (this->map_objects != other.map_objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const Map_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Map_

// alias to use template instance with default allocator
using Map =
  avai_messages::msg::Map_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__MAP__STRUCT_HPP_
