// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from avai_messages:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bboxes'
#include "avai_messages/msg/detail/bounding_box__struct.h"

// Struct defined in msg/BoundingBoxes in the package avai_messages.
typedef struct avai_messages__msg__BoundingBoxes
{
  avai_messages__msg__BoundingBox__Sequence bboxes;
} avai_messages__msg__BoundingBoxes;

// Struct for a sequence of avai_messages__msg__BoundingBoxes.
typedef struct avai_messages__msg__BoundingBoxes__Sequence
{
  avai_messages__msg__BoundingBoxes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} avai_messages__msg__BoundingBoxes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
