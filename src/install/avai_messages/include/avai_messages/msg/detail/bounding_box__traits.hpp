// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from avai_messages:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include "avai_messages/msg/detail/bounding_box__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<avai_messages::msg::BoundingBox>()
{
  return "avai_messages::msg::BoundingBox";
}

template<>
inline const char * name<avai_messages::msg::BoundingBox>()
{
  return "avai_messages/msg/BoundingBox";
}

template<>
struct has_fixed_size<avai_messages::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<avai_messages::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<avai_messages::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
