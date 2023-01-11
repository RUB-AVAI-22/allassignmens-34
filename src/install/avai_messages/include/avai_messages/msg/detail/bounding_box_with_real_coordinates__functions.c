// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from avai_messages:msg/BoundingBoxWithRealCoordinates.idl
// generated code does not contain a copyright notice
#include "avai_messages/msg/detail/bounding_box_with_real_coordinates__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
avai_messages__msg__BoundingBoxWithRealCoordinates__init(avai_messages__msg__BoundingBoxWithRealCoordinates * msg)
{
  if (!msg) {
    return false;
  }
  // image_coords
  // real_coords
  // conf
  // cls
  return true;
}

void
avai_messages__msg__BoundingBoxWithRealCoordinates__fini(avai_messages__msg__BoundingBoxWithRealCoordinates * msg)
{
  if (!msg) {
    return;
  }
  // image_coords
  // real_coords
  // conf
  // cls
}

bool
avai_messages__msg__BoundingBoxWithRealCoordinates__are_equal(const avai_messages__msg__BoundingBoxWithRealCoordinates * lhs, const avai_messages__msg__BoundingBoxWithRealCoordinates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // image_coords
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->image_coords[i] != rhs->image_coords[i]) {
      return false;
    }
  }
  // real_coords
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->real_coords[i] != rhs->real_coords[i]) {
      return false;
    }
  }
  // conf
  if (lhs->conf != rhs->conf) {
    return false;
  }
  // cls
  if (lhs->cls != rhs->cls) {
    return false;
  }
  return true;
}

bool
avai_messages__msg__BoundingBoxWithRealCoordinates__copy(
  const avai_messages__msg__BoundingBoxWithRealCoordinates * input,
  avai_messages__msg__BoundingBoxWithRealCoordinates * output)
{
  if (!input || !output) {
    return false;
  }
  // image_coords
  for (size_t i = 0; i < 4; ++i) {
    output->image_coords[i] = input->image_coords[i];
  }
  // real_coords
  for (size_t i = 0; i < 2; ++i) {
    output->real_coords[i] = input->real_coords[i];
  }
  // conf
  output->conf = input->conf;
  // cls
  output->cls = input->cls;
  return true;
}

avai_messages__msg__BoundingBoxWithRealCoordinates *
avai_messages__msg__BoundingBoxWithRealCoordinates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  avai_messages__msg__BoundingBoxWithRealCoordinates * msg = (avai_messages__msg__BoundingBoxWithRealCoordinates *)allocator.allocate(sizeof(avai_messages__msg__BoundingBoxWithRealCoordinates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(avai_messages__msg__BoundingBoxWithRealCoordinates));
  bool success = avai_messages__msg__BoundingBoxWithRealCoordinates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
avai_messages__msg__BoundingBoxWithRealCoordinates__destroy(avai_messages__msg__BoundingBoxWithRealCoordinates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    avai_messages__msg__BoundingBoxWithRealCoordinates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__init(avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  avai_messages__msg__BoundingBoxWithRealCoordinates * data = NULL;

  if (size) {
    data = (avai_messages__msg__BoundingBoxWithRealCoordinates *)allocator.zero_allocate(size, sizeof(avai_messages__msg__BoundingBoxWithRealCoordinates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = avai_messages__msg__BoundingBoxWithRealCoordinates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        avai_messages__msg__BoundingBoxWithRealCoordinates__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__fini(avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      avai_messages__msg__BoundingBoxWithRealCoordinates__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence *
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * array = (avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence *)allocator.allocate(sizeof(avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__destroy(avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__are_equal(const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * lhs, const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!avai_messages__msg__BoundingBoxWithRealCoordinates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__copy(
  const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * input,
  avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(avai_messages__msg__BoundingBoxWithRealCoordinates);
    avai_messages__msg__BoundingBoxWithRealCoordinates * data =
      (avai_messages__msg__BoundingBoxWithRealCoordinates *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!avai_messages__msg__BoundingBoxWithRealCoordinates__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          avai_messages__msg__BoundingBoxWithRealCoordinates__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!avai_messages__msg__BoundingBoxWithRealCoordinates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
