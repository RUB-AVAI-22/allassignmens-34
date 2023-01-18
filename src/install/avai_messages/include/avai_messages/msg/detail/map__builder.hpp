// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from avai_messages:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__MAP__BUILDER_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__MAP__BUILDER_HPP_

#include "avai_messages/msg/detail/map__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace avai_messages
{

namespace msg
{

namespace builder
{

class Init_Map_map_objects
{
public:
  Init_Map_map_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::avai_messages::msg::Map map_objects(::avai_messages::msg::Map::_map_objects_type arg)
  {
    msg_.map_objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::avai_messages::msg::Map msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::avai_messages::msg::Map>()
{
  return avai_messages::msg::builder::Init_Map_map_objects();
}

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__MAP__BUILDER_HPP_
