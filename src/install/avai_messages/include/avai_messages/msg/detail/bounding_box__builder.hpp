// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from avai_messages:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include "avai_messages/msg/detail/bounding_box__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace avai_messages
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_cls
{
public:
  explicit Init_BoundingBox_cls(::avai_messages::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::avai_messages::msg::BoundingBox cls(::avai_messages::msg::BoundingBox::_cls_type arg)
  {
    msg_.cls = std::move(arg);
    return std::move(msg_);
  }

private:
  ::avai_messages::msg::BoundingBox msg_;
};

class Init_BoundingBox_conf
{
public:
  explicit Init_BoundingBox_conf(::avai_messages::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_cls conf(::avai_messages::msg::BoundingBox::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return Init_BoundingBox_cls(msg_);
  }

private:
  ::avai_messages::msg::BoundingBox msg_;
};

class Init_BoundingBox_coordinates
{
public:
  Init_BoundingBox_coordinates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_conf coordinates(::avai_messages::msg::BoundingBox::_coordinates_type arg)
  {
    msg_.coordinates = std::move(arg);
    return Init_BoundingBox_conf(msg_);
  }

private:
  ::avai_messages::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::avai_messages::msg::BoundingBox>()
{
  return avai_messages::msg::builder::Init_BoundingBox_coordinates();
}

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
