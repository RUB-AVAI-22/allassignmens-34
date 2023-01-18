// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from avai_messages:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__MAP__STRUCT_H_
#define AVAI_MESSAGES__MSG__DETAIL__MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'map_objects'
#include "avai_messages/msg/detail/map_entry__struct.h"

// Struct defined in msg/Map in the package avai_messages.
typedef struct avai_messages__msg__Map
{
  avai_messages__msg__MapEntry__Sequence map_objects;
} avai_messages__msg__Map;

// Struct for a sequence of avai_messages__msg__Map.
typedef struct avai_messages__msg__Map__Sequence
{
  avai_messages__msg__Map * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} avai_messages__msg__Map__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AVAI_MESSAGES__MSG__DETAIL__MAP__STRUCT_H_
