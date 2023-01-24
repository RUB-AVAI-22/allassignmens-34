// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from avai_messages:msg/Map.idl
// generated code does not contain a copyright notice
#include "avai_messages/msg/detail/map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `map_objects`
#include "avai_messages/msg/detail/map_entry__functions.h"

bool
avai_messages__msg__Map__init(avai_messages__msg__Map * msg)
{
  if (!msg) {
    return false;
  }
  // map_objects
  if (!avai_messages__msg__MapEntry__Sequence__init(&msg->map_objects, 0)) {
    avai_messages__msg__Map__fini(msg);
    return false;
  }
  return true;
}

void
avai_messages__msg__Map__fini(avai_messages__msg__Map * msg)
{
  if (!msg) {
    return;
  }
  // map_objects
  avai_messages__msg__MapEntry__Sequence__fini(&msg->map_objects);
}

bool
avai_messages__msg__Map__are_equal(const avai_messages__msg__Map * lhs, const avai_messages__msg__Map * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // map_objects
  if (!avai_messages__msg__MapEntry__Sequence__are_equal(
      &(lhs->map_objects), &(rhs->map_objects)))
  {
    return false;
  }
  return true;
}

bool
avai_messages__msg__Map__copy(
  const avai_messages__msg__Map * input,
  avai_messages__msg__Map * output)
{
  if (!input || !output) {
    return false;
  }
  // map_objects
  if (!avai_messages__msg__MapEntry__Sequence__copy(
      &(input->map_objects), &(output->map_objects)))
  {
    return false;
  }
  return true;
}

avai_messages__msg__Map *
avai_messages__msg__Map__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  avai_messages__msg__Map * msg = (avai_messages__msg__Map *)allocator.allocate(sizeof(avai_messages__msg__Map), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(avai_messages__msg__Map));
  bool success = avai_messages__msg__Map__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
avai_messages__msg__Map__destroy(avai_messages__msg__Map * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    avai_messages__msg__Map__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
avai_messages__msg__Map__Sequence__init(avai_messages__msg__Map__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  avai_messages__msg__Map * data = NULL;

  if (size) {
    data = (avai_messages__msg__Map *)allocator.zero_allocate(size, sizeof(avai_messages__msg__Map), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = avai_messages__msg__Map__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        avai_messages__msg__Map__fini(&data[i - 1]);
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
avai_messages__msg__Map__Sequence__fini(avai_messages__msg__Map__Sequence * array)
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
      avai_messages__msg__Map__fini(&array->data[i]);
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

avai_messages__msg__Map__Sequence *
avai_messages__msg__Map__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  avai_messages__msg__Map__Sequence * array = (avai_messages__msg__Map__Sequence *)allocator.allocate(sizeof(avai_messages__msg__Map__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = avai_messages__msg__Map__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
avai_messages__msg__Map__Sequence__destroy(avai_messages__msg__Map__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    avai_messages__msg__Map__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
avai_messages__msg__Map__Sequence__are_equal(const avai_messages__msg__Map__Sequence * lhs, const avai_messages__msg__Map__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!avai_messages__msg__Map__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
avai_messages__msg__Map__Sequence__copy(
  const avai_messages__msg__Map__Sequence * input,
  avai_messages__msg__Map__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(avai_messages__msg__Map);
    avai_messages__msg__Map * data =
      (avai_messages__msg__Map *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!avai_messages__msg__Map__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          avai_messages__msg__Map__fini(&data[i]);
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
    if (!avai_messages__msg__Map__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
