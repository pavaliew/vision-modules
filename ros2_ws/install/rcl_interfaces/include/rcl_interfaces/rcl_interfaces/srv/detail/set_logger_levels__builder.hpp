// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcl_interfaces:srv/SetLoggerLevels.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__SRV__DETAIL__SET_LOGGER_LEVELS__BUILDER_HPP_
#define RCL_INTERFACES__SRV__DETAIL__SET_LOGGER_LEVELS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rcl_interfaces/srv/detail/set_logger_levels__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rcl_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLoggerLevels_Request_levels
{
public:
  Init_SetLoggerLevels_Request_levels()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcl_interfaces::srv::SetLoggerLevels_Request levels(::rcl_interfaces::srv::SetLoggerLevels_Request::_levels_type arg)
  {
    msg_.levels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcl_interfaces::srv::SetLoggerLevels_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcl_interfaces::srv::SetLoggerLevels_Request>()
{
  return rcl_interfaces::srv::builder::Init_SetLoggerLevels_Request_levels();
}

}  // namespace rcl_interfaces


namespace rcl_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLoggerLevels_Response_results
{
public:
  Init_SetLoggerLevels_Response_results()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcl_interfaces::srv::SetLoggerLevels_Response results(::rcl_interfaces::srv::SetLoggerLevels_Response::_results_type arg)
  {
    msg_.results = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcl_interfaces::srv::SetLoggerLevels_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcl_interfaces::srv::SetLoggerLevels_Response>()
{
  return rcl_interfaces::srv::builder::Init_SetLoggerLevels_Response_results();
}

}  // namespace rcl_interfaces

#endif  // RCL_INTERFACES__SRV__DETAIL__SET_LOGGER_LEVELS__BUILDER_HPP_
