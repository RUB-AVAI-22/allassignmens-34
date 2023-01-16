// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from avai_messages:msg/BoundingBoxesWithRealCoordinates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "avai_messages/msg/detail/bounding_boxes_with_real_coordinates__rosidl_typesupport_introspection_c.h"
#include "avai_messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "avai_messages/msg/detail/bounding_boxes_with_real_coordinates__functions.h"
#include "avai_messages/msg/detail/bounding_boxes_with_real_coordinates__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `bboxes`
#include "avai_messages/msg/bounding_box_with_real_coordinates.h"
// Member `bboxes`
#include "avai_messages/msg/detail/bounding_box_with_real_coordinates__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  avai_messages__msg__BoundingBoxesWithRealCoordinates__init(message_memory);
}

void BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_fini_function(void * message_memory)
{
  avai_messages__msg__BoundingBoxesWithRealCoordinates__fini(message_memory);
}

size_t BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__size_function__BoundingBoxWithRealCoordinates__bboxes(
  const void * untyped_member)
{
  const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * member =
    (const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence *)(untyped_member);
  return member->size;
}

const void * BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxWithRealCoordinates__bboxes(
  const void * untyped_member, size_t index)
{
  const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * member =
    (const avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence *)(untyped_member);
  return &member->data[index];
}

void * BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__get_function__BoundingBoxWithRealCoordinates__bboxes(
  void * untyped_member, size_t index)
{
  avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * member =
    (avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence *)(untyped_member);
  return &member->data[index];
}

bool BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__resize_function__BoundingBoxWithRealCoordinates__bboxes(
  void * untyped_member, size_t size)
{
  avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence * member =
    (avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence *)(untyped_member);
  avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__fini(member);
  return avai_messages__msg__BoundingBoxWithRealCoordinates__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(avai_messages__msg__BoundingBoxesWithRealCoordinates, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bboxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(avai_messages__msg__BoundingBoxesWithRealCoordinates, bboxes),  // bytes offset in struct
    NULL,  // default value
    BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__size_function__BoundingBoxWithRealCoordinates__bboxes,  // size() function pointer
    BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxWithRealCoordinates__bboxes,  // get_const(index) function pointer
    BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__get_function__BoundingBoxWithRealCoordinates__bboxes,  // get(index) function pointer
    BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__resize_function__BoundingBoxWithRealCoordinates__bboxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_members = {
  "avai_messages__msg",  // message namespace
  "BoundingBoxesWithRealCoordinates",  // message name
  2,  // number of fields
  sizeof(avai_messages__msg__BoundingBoxesWithRealCoordinates),
  BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_member_array,  // message members
  BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_init_function,  // function to initialize message memory (memory has to be allocated)
  BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_type_support_handle = {
  0,
  &BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_avai_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, avai_messages, msg, BoundingBoxesWithRealCoordinates)() {
  BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, avai_messages, msg, BoundingBoxWithRealCoordinates)();
  if (!BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_type_support_handle.typesupport_identifier) {
    BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BoundingBoxesWithRealCoordinates__rosidl_typesupport_introspection_c__BoundingBoxesWithRealCoordinates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
