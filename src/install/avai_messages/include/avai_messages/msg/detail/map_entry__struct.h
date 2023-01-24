// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from avai_messages:msg/MapEntry.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__STRUCT_H_
#define AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MapEntry in the package avai_messages.
typedef struct avai_messages__msg__MapEntry
{
  float coordinates[2];
  int8_t cls;
} avai_messages__msg__MapEntry;

// Struct for a sequence of avai_messages__msg__MapEntry.
typedef struct avai_messages__msg__MapEntry__Sequence
{
  avai_messages__msg__MapEntry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} avai_messages__msg__MapEntry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AVAI_MESSAGES__MSG__DETAIL__MAP_ENTRY__STRUCT_H_
