#pragma once
// Logger.hpp — ROS-free log sink for the planning core.
//
// When the real rclcpp/rclcpp.hpp is included first (i.e. in the ROS node
// translation unit), its RCLCPP_* macros are already defined and this file
// becomes a no-op for the macro section.  The g_arms_log_sink declaration is
// still visible to the node, but the node never calls it — it uses the rclcpp
// logger exclusively.  This lets all core .cpp files include Logger.hpp
// unconditionally without breaking the node build.
//
// The RCLCPP_* macros below are drop-in replacements for the real rclcpp macros
// so that no call site in Assembler, ModelLoader, CradleGenerator, etc. needs to
// change.  The logger-object argument (first positional arg) is accepted but
// unused — it exists only for API compatibility.
//
// The actual output goes to g_arms_log_sink, which main() (or the ROS node's
// shim wrapper) installs before calling any pipeline function.  If nothing
// installs it, messages go to stderr.
//
// NOTE: this is a global function pointer.  If the planner is ever called from
// multiple threads concurrently, replace it with an injected parameter on
// Assembler's constructor.

#include <cstdio>
#include <functional>
#include <string>

// Severity levels match Python's logging integers.
namespace arms_log {
    inline constexpr int DEBUG = 10;
    inline constexpr int INFO  = 20;
    inline constexpr int WARN  = 30;
    inline constexpr int ERROR = 40;
    inline constexpr int FATAL = 50;
}

// Install this before any pipeline call.  Defaults to stderr.
extern std::function<void(int /*level*/, const std::string& /*msg*/)> g_arms_log_sink;

// Internal helper — format and dispatch.  Not for direct use.
namespace arms_log_detail {
template<typename... Args>
inline void log(int level, const char* fmt, Args... args)
{
    // snprintf into a fixed buffer; truncate silently beyond 2 KB.
    char buf[2048];
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    std::snprintf(buf, sizeof(buf), fmt, args...);
#pragma GCC diagnostic pop
    if (g_arms_log_sink)
        g_arms_log_sink(level, buf);
}
// Specialisation for zero extra args (avoids -Wformat-zero-variadic-macro-arguments).
inline void log(int level, const char* msg)
{
    if (g_arms_log_sink)
        g_arms_log_sink(level, msg);
}
}  // namespace arms_log_detail

// Drop-in macro replacements.  The first arg (logger object) is consumed but
// not forwarded — it may be "logger()" returning a dummy value or anything else.
// Guard: if rclcpp/rclcpp.hpp was included first it already defined these.
#ifndef RCLCPP_INFO
#define RCLCPP_DEBUG(logger_arg, ...) \
    (static_cast<void>(logger_arg), ::arms_log_detail::log(::arms_log::DEBUG, __VA_ARGS__))
#define RCLCPP_INFO(logger_arg, ...) \
    (static_cast<void>(logger_arg), ::arms_log_detail::log(::arms_log::INFO,  __VA_ARGS__))
#define RCLCPP_WARN(logger_arg, ...) \
    (static_cast<void>(logger_arg), ::arms_log_detail::log(::arms_log::WARN,  __VA_ARGS__))
#define RCLCPP_ERROR(logger_arg, ...) \
    (static_cast<void>(logger_arg), ::arms_log_detail::log(::arms_log::ERROR, __VA_ARGS__))
#define RCLCPP_FATAL(logger_arg, ...) \
    (static_cast<void>(logger_arg), ::arms_log_detail::log(::arms_log::FATAL, __VA_ARGS__))

// The global logger() function that call sites pass as the first arg.
// Returns an int (0) so the static_cast<void> above is legal.
// Guard: rclcpp node TUs define their own logger() via this->get_logger().
inline int logger() { return 0; }
#endif  // RCLCPP_INFO
