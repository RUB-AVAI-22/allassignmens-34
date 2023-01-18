// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from avai_messages:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_

#include "avai_messages/msg/detail/bounding_boxes__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace avai_messages
{

namespace msg
{

namespace builder
{

class Init_BoundingBoxes_bboxes
{
public:
  explicit Init_BoundingBoxes_bboxes(::avai_messages::msg::BoundingBoxes & msg)
  : msg_(msg)
  {}
  ::avai_messages::msg::BoundingBoxes bboxes(::avai_messages::msg::BoundingBoxes::_bboxes_type arg)
  {
    msg_.bboxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxes msg_;
};

class Init_BoundingBoxes_header
{
public:
  Init_BoundingBoxes_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBoxes_bboxes header(::avai_messages::msg::BoundingBoxes::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoundingBoxes_bboxes(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::avai_messages::msg::BoundingBoxes>()
{
  return avai_messages::msg::builder::Init_BoundingBoxes_header();
}

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
