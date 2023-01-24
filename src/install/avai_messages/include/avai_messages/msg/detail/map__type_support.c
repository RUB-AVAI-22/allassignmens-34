// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from avai_messages:msg/Map.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "avai_messages/msg/detail/map__rosidl_typesupport_introspection_c.h"
#include "avai_messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "avai_messages/msg/detail/map__functions.h"
#include "avai_messages/msg/detail/map__struct.h"


// Include directives for member types
// Member `map_objects`
#include "avai_messages/msg/map_entry.h"
// Member `map_objects`
#include "avai_messages/msg/detail/map_entry__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Map__rosidl_typesupport_introspection_c__Map_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  avai_messages__msg__Map__init(message_memory);
}

void Map__rosidl_typesupport_introspection_c__Map_fini_function(void * message_memory)
{
  avai_messages__msg__Map__fini(message_memory);
}

size_t Map__rosidl_typesupport_introspection_c__size_function__MapEntry__map_objects(
  const void * untyped_member)
{
  const avai_messages__msg__MapEntry__Sequence * member =
    (const avai_messages__msg__MapEntry__Sequence *)(untyped_member);
  return member->size;
}

const void * Map__rosidl_typesupport_introspection_c__get_const_function__MapEntry__map_objects(
  const void * untyped_member, size_t index)
{
  const avai_messages__msg__MapEntry__Sequence * member =
    (const avai_messages__msg__MapEntry__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Map__rosidl_typesupport_introspection_c__get_function__MapEntry__map_objects(
  void * untyped_member, size_t index)
{
  avai_messages__msg__MapEntry__Sequence * member =
    (avai_messages__msg__MapEntry__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Map__rosidl_typesupport_introspection_c__resize_function__MapEntry__map_objects(
  void * untyped_member, size_t size)
{
  avai_messages__msg__MapEntry__Sequence * member =
    (avai_messages__msg__MapEntry__Sequence *)(untyped_member);
  avai_messages__msg__MapEntry__Sequence__fini(member);
  return avai_messages__msg__MapEntry__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Map__rosidl_typesupport_introspection_c__Map_message_member_array[1] = {
  {
    "map_objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(avai_messages__msg__Map, map_objects),  // bytes offset in struct
    NULL,  // default value
    Map__rosidl_typesupport_introspection_c__size_function__MapEntry__map_objects,  // size() function pointer
    Map__rosidl_typesupport_introspection_c__get_const_function__MapEntry__map_objects,  // get_const(index) function pointer
    Map__rosidl_typesupport_introspection_c__get_function__MapEntry__map_objects,  // get(index) function pointer
    Map__rosidl_typesupport_introspection_c__resize_function__MapEntry__map_objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Map__rosidl_typesupport_introspection_c__Map_message_members = {
  "avai_messages__msg",  // message namespace
  "Map",  // message name
  1,  // number of fields
  sizeof(avai_messages__msg__Map),
  Map__rosidl_typesupport_introspection_c__Map_message_member_array,  // message members
  Map__rosidl_typesupport_introspection_c__Map_init_function,  // function to initialize message memory (memory has to be allocated)
  Map__rosidl_typesupport_introspection_c__Map_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle = {
  0,
  &Map__rosidl_typesupport_introspection_c__Map_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_avai_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, avai_messages, msg, Map)() {
  Map__rosidl_typesupport_introspection_c__Map_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, avai_messages, msg, MapEntry)();
  if (!Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle.typesupport_identifier) {
    Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Map__rosidl_typesupport_introspection_c__Map_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
