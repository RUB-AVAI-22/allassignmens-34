// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from avai_messages:msg/BoundingBoxesWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__TRAITS_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__TRAITS_HPP_

#include "avai_messages/msg/detail/bounding_boxes_with_real_coordinates__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<avai_messages::msg::BoundingBoxesWithRealCoordinates>()
{
  return "avai_messages::msg::BoundingBoxesWithRealCoordinates";
}

template<>
inline const char * name<avai_messages::msg::BoundingBoxesWithRealCoordinates>()
{
  return "avai_messages/msg/BoundingBoxesWithRealCoordinates";
}

template<>
struct has_fixed_size<avai_messages::msg::BoundingBoxesWithRealCoordinates>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<avai_messages::msg::BoundingBoxesWithRealCoordinates>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<avai_messages::msg::BoundingBoxesWithRealCoordinates>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__TRAITS_HPP_
