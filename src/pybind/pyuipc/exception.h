#pragma once
#include <string>
#include <string_view>
#include <uipc/common/exception.h>
#include <fmt/format.h>

namespace pyuipc
{
class PyException : public uipc::Exception
{
  public:
    using uipc::Exception::Exception;
};

namespace detail
{
    std::string string_with_source_location(std::string_view msg,
                                            std::string_view path,
                                            std::size_t      line);

    void assert_with_source_location(bool             condition,
                                     std::string_view condition_str,
                                     std::string_view msg,
                                     std::string_view path,
                                     std::size_t      line);
}  // namespace detail
}  // namespace pyuipc

#define PYUIPC_MSG(...)                                                        \
    ::pyuipc::detail::string_with_source_location(fmt::format(__VA_ARGS__), __FILE__, __LINE__)

#define PYUIPC_ASSERT(condition, ...)                                          \
    ::pyuipc::detail::assert_with_source_location(                             \
        (condition), #condition, fmt::format(__VA_ARGS__), __FILE__, __LINE__)

