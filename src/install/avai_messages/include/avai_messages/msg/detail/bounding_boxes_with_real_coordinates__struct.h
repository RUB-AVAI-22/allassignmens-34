// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from avai_messages:msg/BoundingBoxesWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__STRUCT_H_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__STRUCT_H_

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
#include "avai_messages/msg/detail/bounding_box_with_real_coordinates__struct.h"

// Struct defined in msg/BoundingBoxesWithRealCoordinates in the package avai_messages.
typedef struct avai_messages__msg__BoundingBoxesWithRealCoordinates
{
  avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence bboxes;
} avai_messages__msg__BoundingBoxesWithRealCoordinates;

// Struct for a sequence of avai_messages__msg__BoundingBoxesWithRealCoordinates.
typedef struct avai_messages__msg__BoundingBoxesWithRealCoordinates__Sequence
{
  avai_messages__msg__BoundingBoxesWithRealCoordinates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} avai_messages__msg__BoundingBoxesWithRealCoordinates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOXES_WITH_REAL_COORDINATES__STRUCT_H_
