// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from avai_messages:msg/BoundingBoxWithRealCoordinates.idl
// generated code does not contain a copyright notice

#ifndef AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__FUNCTIONS_H_
#define AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "avai_messages/msg/rosidl_generator_c__visibility_control.h"

#include "avai_messages/msg/detail/bounding_box_with_real_coordinates__struct.h"

/// Initialize msg/BoundingBoxWithRealCoordinates message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * avai_messages__msg__BoundingBoxWithRealCoordinates
 * )) before or use
 * avai_messages__msg__BoundingBoxWithRealCoordinates__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
bool
avai_messages__msg__BoundingBoxWithRealCoordinates__init(avai_messages__msg__BoundingBoxWithRealCoordinates * msg);

/// Finalize msg/BoundingBoxWithRealCoordinates message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
void
avai_messages__msg__BoundingBoxWithRealCoordinates__fini(avai_messages__msg__BoundingBoxWithRealCoordinates * msg);

/// Create msg/BoundingBoxWithRealCoordinates message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * avai_messages__msg__BoundingBoxWithRealCoordinates__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
avai_messages__msg__BoundingBoxWithRealCoordinates *
avai_messages__msg__BoundingBoxWithRealCoordinates__create();

/// Destroy msg/BoundingBoxWithRealCoordinates message.
/**
 * It calls
 * avai_messages__msg__BoundingBoxWithRealCoordinates__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
void
avai_messages__msg__BoundingBoxWithRealCoordinates__destroy(avai_messages__msg__BoundingBoxWithRealCoordinates * msg);

/// Check for msg/BoundingBoxWithRealCoordinates message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
bool
avai_messages__msg__BoundingBoxWithRealCoordinates__are_equal(const avai_messages__msg__BoundingBoxWithRealCoordinates * lhs, const avai_messages__msg__BoundingBoxWithRealCoordinates * rhs);

/// Copy a msg/BoundingBoxWithRealCoordinates message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
bool
avai_messages__msg__BoundingBoxWithRealCoordinates__copy(
  const avai_messages__msg__BoundingBoxWithRealCoordinates * input,
  avai_messages__msg__BoundingBoxWithRealCoordinates * output);

/// Initialize array of msg/BoundingBoxWithRealCoordinates messages.
/**
 * It allocates the memory for the number of elements and calls
 * avai_messages__msg__BoundingBoxWithRealCoordinates__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
bool
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__init(avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * array, size_t size);

/// Finalize array of msg/BoundingBoxWithRealCoordinates messages.
/**
 * It calls
 * avai_messages__msg__BoundingBoxWithRealCoordinates__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
void
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__fini(avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * array);

/// Create array of msg/BoundingBoxWithRealCoordinates messages.
/**
 * It allocates the memory for the array and calls
 * avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence *
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__create(size_t size);

/// Destroy array of msg/BoundingBoxWithRealCoordinates messages.
/**
 * It calls
 * avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
void
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__destroy(avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * array);

/// Check for msg/BoundingBoxWithRealCoordinates message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
bool
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__are_equal(const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * lhs, const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * rhs);

/// Copy an array of msg/BoundingBoxWithRealCoordinates messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_avai_messages
bool
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__copy(
  const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * input,
  avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AVAI_MESSAGES__MSG__DETAIL__BOUNDING_BOX_WITH_REAL_COORDINATES__FUNCTIONS_H_
