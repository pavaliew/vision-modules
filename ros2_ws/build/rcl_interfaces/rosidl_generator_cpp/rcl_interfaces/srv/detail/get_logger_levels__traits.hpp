// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:srv/GetLoggerLevels.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__SRV__DETAIL__GET_LOGGER_LEVELS__TRAITS_HPP_
#define RCL_INTERFACES__SRV__DETAIL__GET_LOGGER_LEVELS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rcl_interfaces/srv/detail/get_logger_levels__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rcl_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetLoggerLevels_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: names
  {
    if (msg.names.size() == 0) {
      out << "names: []";
    } else {
      out << "names: [";
      size_t pending_items = msg.names.size();
      for (auto item : msg.names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetLoggerLevels_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.names.size() == 0) {
      out << "names: []\n";
    } else {
      out << "names:\n";
      for (auto item : msg.names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetLoggerLevels_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rcl_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rcl_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rcl_interfaces::srv::GetLoggerLevels_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rcl_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rcl_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rcl_interfaces::srv::GetLoggerLevels_Request & msg)
{
  return rcl_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rcl_interfaces::srv::GetLoggerLevels_Request>()
{
  return "rcl_interfaces::srv::GetLoggerLevels_Request";
}

template<>
inline const char * name<rcl_interfaces::srv::GetLoggerLevels_Request>()
{
  return "rcl_interfaces/srv/GetLoggerLevels_Request";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetLoggerLevels_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetLoggerLevels_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::GetLoggerLevels_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'levels'
#include "rcl_interfaces/msg/detail/logger_level__traits.hpp"

namespace rcl_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetLoggerLevels_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: levels
  {
    if (msg.levels.size() == 0) {
      out << "levels: []";
    } else {
      out << "levels: [";
      size_t pending_items = msg.levels.size();
      for (auto item : msg.levels) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetLoggerLevels_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: levels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.levels.size() == 0) {
      out << "levels: []\n";
    } else {
      out << "levels:\n";
      for (auto item : msg.levels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetLoggerLevels_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rcl_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rcl_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rcl_interfaces::srv::GetLoggerLevels_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rcl_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rcl_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rcl_interfaces::srv::GetLoggerLevels_Response & msg)
{
  return rcl_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rcl_interfaces::srv::GetLoggerLevels_Response>()
{
  return "rcl_interfaces::srv::GetLoggerLevels_Response";
}

template<>
inline const char * name<rcl_interfaces::srv::GetLoggerLevels_Response>()
{
  return "rcl_interfaces/srv/GetLoggerLevels_Response";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetLoggerLevels_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetLoggerLevels_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::GetLoggerLevels_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcl_interfaces::srv::GetLoggerLevels>()
{
  return "rcl_interfaces::srv::GetLoggerLevels";
}

template<>
inline const char * name<rcl_interfaces::srv::GetLoggerLevels>()
{
  return "rcl_interfaces/srv/GetLoggerLevels";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetLoggerLevels>
  : std::integral_constant<
    bool,
    has_fixed_size<rcl_interfaces::srv::GetLoggerLevels_Request>::value &&
    has_fixed_size<rcl_interfaces::srv::GetLoggerLevels_Response>::value
  >
{
};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetLoggerLevels>
  : std::integral_constant<
    bool,
    has_bounded_size<rcl_interfaces::srv::GetLoggerLevels_Request>::value &&
    has_bounded_size<rcl_interfaces::srv::GetLoggerLevels_Response>::value
  >
{
};

template<>
struct is_service<rcl_interfaces::srv::GetLoggerLevels>
  : std::true_type
{
};

template<>
struct is_service_request<rcl_interfaces::srv::GetLoggerLevels_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rcl_interfaces::srv::GetLoggerLevels_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__SRV__DETAIL__GET_LOGGER_LEVELS__TRAITS_HPP_
