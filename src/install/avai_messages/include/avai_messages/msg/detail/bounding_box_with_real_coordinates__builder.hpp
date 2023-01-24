// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from avai_messages:msg/BoundingBoxWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__BUILDER_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__BUILDER_HPP_

#include "avai_messages/msg/detail/bounding_box_with_real_coordinates__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace avai_messages
{

namespace msg
{

namespace builder
{

class Init_BoundingBoxWithRealCoordinates_real_coords
{
public:
  explicit Init_BoundingBoxWithRealCoordinates_real_coords(::avai_messages::msg::BoundingBoxWithRealCoordinates & msg)
  : msg_(msg)
  {}
  ::avai_messages::msg::BoundingBoxWithRealCoordinates real_coords(::avai_messages::msg::BoundingBoxWithRealCoordinates::_real_coords_type arg)
  {
    msg_.real_coords = std::move(arg);
    return std::move(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxWithRealCoordinates msg_;
};

class Init_BoundingBoxWithRealCoordinates_cls
{
public:
  explicit Init_BoundingBoxWithRealCoordinates_cls(::avai_messages::msg::BoundingBoxWithRealCoordinates & msg)
  : msg_(msg)
  {}
  Init_BoundingBoxWithRealCoordinates_real_coords cls(::avai_messages::msg::BoundingBoxWithRealCoordinates::_cls_type arg)
  {
    msg_.cls = std::move(arg);
    return Init_BoundingBoxWithRealCoordinates_real_coords(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxWithRealCoordinates msg_;
};

class Init_BoundingBoxWithRealCoordinates_conf
{
public:
  explicit Init_BoundingBoxWithRealCoordinates_conf(::avai_messages::msg::BoundingBoxWithRealCoordinates & msg)
  : msg_(msg)
  {}
  Init_BoundingBoxWithRealCoordinates_cls conf(::avai_messages::msg::BoundingBoxWithRealCoordinates::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return Init_BoundingBoxWithRealCoordinates_cls(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxWithRealCoordinates msg_;
};

class Init_BoundingBoxWithRealCoordinates_image_coords
{
public:
  Init_BoundingBoxWithRealCoordinates_image_coords()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBoxWithRealCoordinates_conf image_coords(::avai_messages::msg::BoundingBoxWithRealCoordinates::_image_coords_type arg)
  {
    msg_.image_coords = std::move(arg);
    return Init_BoundingBoxWithRealCoordinates_conf(msg_);
  }

private:
  ::avai_messages::msg::BoundingBoxWithRealCoordinates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::avai_messages::msg::BoundingBoxWithRealCoordinates>()
{
  return avai_messages::msg::builder::Init_BoundingBoxWithRealCoordinates_image_coords();
}

}  // namespace avai_messages

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__BUILDER_HPP_
