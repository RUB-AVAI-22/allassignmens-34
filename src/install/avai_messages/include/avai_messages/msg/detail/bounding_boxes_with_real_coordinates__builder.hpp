// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from avai_messages:msg/BoundingBoxesWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__BUILDER_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__BUILDER_HPP_

#include "avai_messages/msg/detail/bounding_boxes_with_real_coordinates__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace avai_messages
{

namespace msg
{

namespace builder
{

class Init_BoundingBoxesWithRealCoordinates_bboxes
{
public:
  explicit Init_BoundingBoxesWithRealCoordinates_bboxes(::avai_messages::msg::BoundingBoxesWithRealCoordinates & msg)
  : msg_(msg)
  {}
  ::avai_messages::msg::BoundingBoxesWithRealCoordinates bboxes(::avai_messages::msg::BoundingBoxesWithRealCoordinates::_bboxes_type arg)
  {
    msg_.bboxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxesWithRealCoordinates msg_;
};

class Init_BoundingBoxesWithRealCoordinates_header
{
public:
  Init_BoundingBoxesWithRealCoordinates_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBoxesWithRealCoordinates_bboxes header(::avai_messages::msg::BoundingBoxesWithRealCoordinates::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoundingBoxesWithRealCoordinates_bboxes(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxesWithRealCoordinates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::avai_messages::msg::BoundingBoxesWithRealCoordinates>()
{
  return avai_messages::msg::builder::Init_BoundingBoxesWithRealCoordinates_header();
}

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__BUILDER_HPP_
