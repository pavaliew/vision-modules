// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcl_interfaces:srv/GetLoggerLevels.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__SRV__DETAIL__GET_LOGGER_LEVELS__BUILDER_HPP_
#define RCL_INTERFACES__SRV__DETAIL__GET_LOGGER_LEVELS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rcl_interfaces/srv/detail/get_logger_levels__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rcl_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetLoggerLevels_Request_names
{
public:
  Init_GetLoggerLevels_Request_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcl_interfaces::srv::GetLoggerLevels_Request names(::rcl_interfaces::srv::GetLoggerLevels_Request::_names_type arg)
  {
    msg_.names = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcl_interfaces::srv::GetLoggerLevels_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcl_interfaces::srv::GetLoggerLevels_Request>()
{
  return rcl_interfaces::srv::builder::Init_GetLoggerLevels_Request_names();
}

}  // namespace rcl_interfaces


namespace rcl_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetLoggerLevels_Response_levels
{
public:
  Init_GetLoggerLevels_Response_levels()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcl_interfaces::srv::GetLoggerLevels_Response levels(::rcl_interfaces::srv::GetLoggerLevels_Response::_levels_type arg)
  {
    msg_.levels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcl_interfaces::srv::GetLoggerLevels_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcl_interfaces::srv::GetLoggerLevels_Response>()
{
  return rcl_interfaces::srv::builder::Init_GetLoggerLevels_Response_levels();
}

}  // namespace rcl_interfaces

#endif  // RCL_INTERFACES__SRV__DETAIL__GET_LOGGER_LEVELS__BUILDER_HPP_
