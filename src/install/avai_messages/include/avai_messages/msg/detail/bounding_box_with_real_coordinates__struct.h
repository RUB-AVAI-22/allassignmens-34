// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from avai_messages:msg/BoundingBoxWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__STRUCT_H_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/BoundingBoxWithRealCoordinates in the package avai_messages.
typedef struct avai_messages__msg__BoundingBoxWithRealCoordinates
{
  float image_coords[4];
  float real_coords[2];
  float conf;
  float cls;
} avai_messages__msg__BoundingBoxWithRealCoordinates;

// Struct for a sequence of avai_messages__msg__BoundingBoxWithRealCoordinates.
typedef struct avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence
{
  avai_messages__msg__BoundingBoxWithRealCoordinates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__STRUCT_H_
