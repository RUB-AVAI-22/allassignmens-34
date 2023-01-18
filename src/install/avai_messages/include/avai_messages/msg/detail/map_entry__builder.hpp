// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from avai_messages:msg/MapEntry.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__BUILDER_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__BUILDER_HPP_

#include "avai_messages/msg/detail/map_entry__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace avai_messages
{

namespace msg
{

namespace builder
{

class Init_MapEntry_cls
{
public:
  explicit Init_MapEntry_cls(::avai_messages::msg::MapEntry & msg)
  : msg_(msg)
  {}
  ::avai_messages::msg::MapEntry cls(::avai_messages::msg::MapEntry::_cls_type arg)
  {
    msg_.cls = std::move(arg);
    return std::move(msg_);
  }

private:
  ::avai_messages::msg::MapEntry msg_;
};

class Init_MapEntry_coordinates
{
public:
  Init_MapEntry_coordinates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapEntry_cls coordinates(::avai_messages::msg::MapEntry::_coordinates_type arg)
  {
    msg_.coordinates = std::move(arg);
    return Init_MapEntry_cls(msg_);
  }

private:
  ::avai_messages::msg::MapEntry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::avai_messages::msg::MapEntry>()
{
  return avai_messages::msg::builder::Init_MapEntry_coordinates();
}

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__BUILDER_HPP_
