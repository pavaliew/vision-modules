// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rcl_interfaces:srv/SetLoggerLevels.idl
// generated code does not contain a copyright notice
#include "rcl_interfaces/srv/detail/set_logger_levels__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rcl_interfaces/srv/detail/set_logger_levels__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace rcl_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rcl_interfaces::msg::LoggerLevel &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rcl_interfaces::msg::LoggerLevel &);
size_t get_serialized_size(
  const rcl_interfaces::msg::LoggerLevel &,
  size_t current_alignment);
size_t
max_serialized_size_LoggerLevel(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rcl_interfaces


namespace rcl_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
cdr_serialize(
  const rcl_interfaces::srv::SetLoggerLevels_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: levels
  {
    size_t size = ros_message.levels.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rcl_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.levels[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rcl_interfaces::srv::SetLoggerLevels_Request & ros_message)
{
  // Member: levels
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.levels.resize(size);
    for (size_t i = 0; i < size; i++) {
      rcl_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.levels[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
get_serialized_size(
  const rcl_interfaces::srv::SetLoggerLevels_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: levels
  {
    size_t array_size = ros_message.levels.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rcl_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.levels[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
max_serialized_size_SetLoggerLevels_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: levels
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        rcl_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_LoggerLevel(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rcl_interfaces::srv::SetLoggerLevels_Request;
    is_plain =
      (
      offsetof(DataType, levels) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SetLoggerLevels_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rcl_interfaces::srv::SetLoggerLevels_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetLoggerLevels_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rcl_interfaces::srv::SetLoggerLevels_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetLoggerLevels_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rcl_interfaces::srv::SetLoggerLevels_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetLoggerLevels_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SetLoggerLevels_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SetLoggerLevels_Request__callbacks = {
  "rcl_interfaces::srv",
  "SetLoggerLevels_Request",
  _SetLoggerLevels_Request__cdr_serialize,
  _SetLoggerLevels_Request__cdr_deserialize,
  _SetLoggerLevels_Request__get_serialized_size,
  _SetLoggerLevels_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetLoggerLevels_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetLoggerLevels_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rcl_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rcl_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<rcl_interfaces::srv::SetLoggerLevels_Request>()
{
  return &rcl_interfaces::srv::typesupport_fastrtps_cpp::_SetLoggerLevels_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rcl_interfaces, srv, SetLoggerLevels_Request)() {
  return &rcl_interfaces::srv::typesupport_fastrtps_cpp::_SetLoggerLevels_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace rcl_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rcl_interfaces::msg::SetLoggerLevelsResult &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rcl_interfaces::msg::SetLoggerLevelsResult &);
size_t get_serialized_size(
  const rcl_interfaces::msg::SetLoggerLevelsResult &,
  size_t current_alignment);
size_t
max_serialized_size_SetLoggerLevelsResult(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rcl_interfaces


namespace rcl_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
cdr_serialize(
  const rcl_interfaces::srv::SetLoggerLevels_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: results
  {
    size_t size = ros_message.results.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rcl_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.results[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rcl_interfaces::srv::SetLoggerLevels_Response & ros_message)
{
  // Member: results
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.results.resize(size);
    for (size_t i = 0; i < size; i++) {
      rcl_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.results[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
get_serialized_size(
  const rcl_interfaces::srv::SetLoggerLevels_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: results
  {
    size_t array_size = ros_message.results.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rcl_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.results[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rcl_interfaces
max_serialized_size_SetLoggerLevels_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: results
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        rcl_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_SetLoggerLevelsResult(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rcl_interfaces::srv::SetLoggerLevels_Response;
    is_plain =
      (
      offsetof(DataType, results) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SetLoggerLevels_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rcl_interfaces::srv::SetLoggerLevels_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetLoggerLevels_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rcl_interfaces::srv::SetLoggerLevels_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetLoggerLevels_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rcl_interfaces::srv::SetLoggerLevels_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetLoggerLevels_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SetLoggerLevels_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SetLoggerLevels_Response__callbacks = {
  "rcl_interfaces::srv",
  "SetLoggerLevels_Response",
  _SetLoggerLevels_Response__cdr_serialize,
  _SetLoggerLevels_Response__cdr_deserialize,
  _SetLoggerLevels_Response__get_serialized_size,
  _SetLoggerLevels_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetLoggerLevels_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetLoggerLevels_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rcl_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rcl_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<rcl_interfaces::srv::SetLoggerLevels_Response>()
{
  return &rcl_interfaces::srv::typesupport_fastrtps_cpp::_SetLoggerLevels_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rcl_interfaces, srv, SetLoggerLevels_Response)() {
  return &rcl_interfaces::srv::typesupport_fastrtps_cpp::_SetLoggerLevels_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace rcl_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _SetLoggerLevels__callbacks = {
  "rcl_interfaces::srv",
  "SetLoggerLevels",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rcl_interfaces, srv, SetLoggerLevels_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rcl_interfaces, srv, SetLoggerLevels_Response)(),
};

static rosidl_service_type_support_t _SetLoggerLevels__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetLoggerLevels__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rcl_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rcl_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<rcl_interfaces::srv::SetLoggerLevels>()
{
  return &rcl_interfaces::srv::typesupport_fastrtps_cpp::_SetLoggerLevels__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rcl_interfaces, srv, SetLoggerLevels)() {
  return &rcl_interfaces::srv::typesupport_fastrtps_cpp::_SetLoggerLevels__handle;
}

#ifdef __cplusplus
}
#endif
