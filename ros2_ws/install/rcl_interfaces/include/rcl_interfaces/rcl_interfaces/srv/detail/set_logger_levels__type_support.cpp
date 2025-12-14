// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rcl_interfaces:srv/SetLoggerLevels.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rcl_interfaces/srv/detail/set_logger_levels__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcl_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetLoggerLevels_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcl_interfaces::srv::SetLoggerLevels_Request(_init);
}

void SetLoggerLevels_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcl_interfaces::srv::SetLoggerLevels_Request *>(message_memory);
  typed_message->~SetLoggerLevels_Request();
}

size_t size_function__SetLoggerLevels_Request__levels(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rcl_interfaces::msg::LoggerLevel> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetLoggerLevels_Request__levels(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rcl_interfaces::msg::LoggerLevel> *>(untyped_member);
  return &member[index];
}

void * get_function__SetLoggerLevels_Request__levels(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rcl_interfaces::msg::LoggerLevel> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetLoggerLevels_Request__levels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const rcl_interfaces::msg::LoggerLevel *>(
    get_const_function__SetLoggerLevels_Request__levels(untyped_member, index));
  auto & value = *reinterpret_cast<rcl_interfaces::msg::LoggerLevel *>(untyped_value);
  value = item;
}

void assign_function__SetLoggerLevels_Request__levels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<rcl_interfaces::msg::LoggerLevel *>(
    get_function__SetLoggerLevels_Request__levels(untyped_member, index));
  const auto & value = *reinterpret_cast<const rcl_interfaces::msg::LoggerLevel *>(untyped_value);
  item = value;
}

void resize_function__SetLoggerLevels_Request__levels(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rcl_interfaces::msg::LoggerLevel> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetLoggerLevels_Request_message_member_array[1] = {
  {
    "levels",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rcl_interfaces::msg::LoggerLevel>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcl_interfaces::srv::SetLoggerLevels_Request, levels),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetLoggerLevels_Request__levels,  // size() function pointer
    get_const_function__SetLoggerLevels_Request__levels,  // get_const(index) function pointer
    get_function__SetLoggerLevels_Request__levels,  // get(index) function pointer
    fetch_function__SetLoggerLevels_Request__levels,  // fetch(index, &value) function pointer
    assign_function__SetLoggerLevels_Request__levels,  // assign(index, value) function pointer
    resize_function__SetLoggerLevels_Request__levels  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetLoggerLevels_Request_message_members = {
  "rcl_interfaces::srv",  // message namespace
  "SetLoggerLevels_Request",  // message name
  1,  // number of fields
  sizeof(rcl_interfaces::srv::SetLoggerLevels_Request),
  SetLoggerLevels_Request_message_member_array,  // message members
  SetLoggerLevels_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetLoggerLevels_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetLoggerLevels_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetLoggerLevels_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace rcl_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcl_interfaces::srv::SetLoggerLevels_Request>()
{
  return &::rcl_interfaces::srv::rosidl_typesupport_introspection_cpp::SetLoggerLevels_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcl_interfaces, srv, SetLoggerLevels_Request)() {
  return &::rcl_interfaces::srv::rosidl_typesupport_introspection_cpp::SetLoggerLevels_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcl_interfaces/srv/detail/set_logger_levels__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcl_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetLoggerLevels_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcl_interfaces::srv::SetLoggerLevels_Response(_init);
}

void SetLoggerLevels_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcl_interfaces::srv::SetLoggerLevels_Response *>(message_memory);
  typed_message->~SetLoggerLevels_Response();
}

size_t size_function__SetLoggerLevels_Response__results(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rcl_interfaces::msg::SetLoggerLevelsResult> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetLoggerLevels_Response__results(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rcl_interfaces::msg::SetLoggerLevelsResult> *>(untyped_member);
  return &member[index];
}

void * get_function__SetLoggerLevels_Response__results(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rcl_interfaces::msg::SetLoggerLevelsResult> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetLoggerLevels_Response__results(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const rcl_interfaces::msg::SetLoggerLevelsResult *>(
    get_const_function__SetLoggerLevels_Response__results(untyped_member, index));
  auto & value = *reinterpret_cast<rcl_interfaces::msg::SetLoggerLevelsResult *>(untyped_value);
  value = item;
}

void assign_function__SetLoggerLevels_Response__results(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<rcl_interfaces::msg::SetLoggerLevelsResult *>(
    get_function__SetLoggerLevels_Response__results(untyped_member, index));
  const auto & value = *reinterpret_cast<const rcl_interfaces::msg::SetLoggerLevelsResult *>(untyped_value);
  item = value;
}

void resize_function__SetLoggerLevels_Response__results(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rcl_interfaces::msg::SetLoggerLevelsResult> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetLoggerLevels_Response_message_member_array[1] = {
  {
    "results",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rcl_interfaces::msg::SetLoggerLevelsResult>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcl_interfaces::srv::SetLoggerLevels_Response, results),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetLoggerLevels_Response__results,  // size() function pointer
    get_const_function__SetLoggerLevels_Response__results,  // get_const(index) function pointer
    get_function__SetLoggerLevels_Response__results,  // get(index) function pointer
    fetch_function__SetLoggerLevels_Response__results,  // fetch(index, &value) function pointer
    assign_function__SetLoggerLevels_Response__results,  // assign(index, value) function pointer
    resize_function__SetLoggerLevels_Response__results  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetLoggerLevels_Response_message_members = {
  "rcl_interfaces::srv",  // message namespace
  "SetLoggerLevels_Response",  // message name
  1,  // number of fields
  sizeof(rcl_interfaces::srv::SetLoggerLevels_Response),
  SetLoggerLevels_Response_message_member_array,  // message members
  SetLoggerLevels_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetLoggerLevels_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetLoggerLevels_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetLoggerLevels_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace rcl_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcl_interfaces::srv::SetLoggerLevels_Response>()
{
  return &::rcl_interfaces::srv::rosidl_typesupport_introspection_cpp::SetLoggerLevels_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcl_interfaces, srv, SetLoggerLevels_Response)() {
  return &::rcl_interfaces::srv::rosidl_typesupport_introspection_cpp::SetLoggerLevels_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "rcl_interfaces/srv/detail/set_logger_levels__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace rcl_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SetLoggerLevels_service_members = {
  "rcl_interfaces::srv",  // service namespace
  "SetLoggerLevels",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<rcl_interfaces::srv::SetLoggerLevels>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SetLoggerLevels_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetLoggerLevels_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace rcl_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rcl_interfaces::srv::SetLoggerLevels>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::rcl_interfaces::srv::rosidl_typesupport_introspection_cpp::SetLoggerLevels_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rcl_interfaces::srv::SetLoggerLevels_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rcl_interfaces::srv::SetLoggerLevels_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcl_interfaces, srv, SetLoggerLevels)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<rcl_interfaces::srv::SetLoggerLevels>();
}

#ifdef __cplusplus
}
#endif
