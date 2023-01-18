// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from avai_messages:msg/BoundingBoxWithRealCoordinates.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "avai_messages/msg/detail/bounding_box_with_real_coordinates__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace avai_messages
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BoundingBoxWithRealCoordinates_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) avai_messages::msg::BoundingBoxWithRealCoordinates(_init);
}

void BoundingBoxWithRealCoordinates_fini_function(void * message_memory)
{
  auto typed_message = static_cast<avai_messages::msg::BoundingBoxWithRealCoordinates *>(message_memory);
  typed_message->~BoundingBoxWithRealCoordinates();
}

size_t size_function__BoundingBoxWithRealCoordinates__image_coords(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__BoundingBoxWithRealCoordinates__image_coords(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__BoundingBoxWithRealCoordinates__image_coords(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

size_t size_function__BoundingBoxWithRealCoordinates__real_coords(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__BoundingBoxWithRealCoordinates__real_coords(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__BoundingBoxWithRealCoordinates__real_coords(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BoundingBoxWithRealCoordinates_message_member_array[4] = {
  {
    "image_coords",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(avai_messages::msg::BoundingBoxWithRealCoordinates, image_coords),  // bytes offset in struct
    nullptr,  // default value
    size_function__BoundingBoxWithRealCoordinates__image_coords,  // size() function pointer
    get_const_function__BoundingBoxWithRealCoordinates__image_coords,  // get_const(index) function pointer
    get_function__BoundingBoxWithRealCoordinates__image_coords,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "conf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(avai_messages::msg::BoundingBoxWithRealCoordinates, conf),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cls",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(avai_messages::msg::BoundingBoxWithRealCoordinates, cls),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "real_coords",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(avai_messages::msg::BoundingBoxWithRealCoordinates, real_coords),  // bytes offset in struct
    nullptr,  // default value
    size_function__BoundingBoxWithRealCoordinates__real_coords,  // size() function pointer
    get_const_function__BoundingBoxWithRealCoordinates__real_coords,  // get_const(index) function pointer
    get_function__BoundingBoxWithRealCoordinates__real_coords,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BoundingBoxWithRealCoordinates_message_members = {
  "avai_messages::msg",  // message namespace
  "BoundingBoxWithRealCoordinates",  // message name
  4,  // number of fields
  sizeof(avai_messages::msg::BoundingBoxWithRealCoordinates),
  BoundingBoxWithRealCoordinates_message_member_array,  // message members
  BoundingBoxWithRealCoordinates_init_function,  // function to initialize message memory (memory has to be allocated)
  BoundingBoxWithRealCoordinates_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BoundingBoxWithRealCoordinates_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BoundingBoxWithRealCoordinates_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace avai_messages


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<avai_messages::msg::BoundingBoxWithRealCoordinates>()
{
  return &::avai_messages::msg::rosidl_typesupport_introspection_cpp::BoundingBoxWithRealCoordinates_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, avai_messages, msg, BoundingBoxWithRealCoordinates)() {
  return &::avai_messages::msg::rosidl_typesupport_introspection_cpp::BoundingBoxWithRealCoordinates_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
