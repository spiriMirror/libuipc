#pragma once
#include <pybind11/pybind11.h>
#include <functional>
#include <uipc/common/type_define.h>
#include <uipc/common/exception.h>

namespace pyuipc
{
using namespace uipc;
namespace py = pybind11;

extern py::module& top_module();

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

class PyException : public uipc::Exception
{
  public:
    using uipc::Exception::Exception;
};
}  // namespace pyuipc

#define PYUIPC_MSG(...)                                                        \
    ::pyuipc::detail::string_with_source_location(fmt::format(__VA_ARGS__), __FILE__, __LINE__)

#define PYUIPC_ASSERT(condition, ...)                                          \
    ::pyuipc::detail::assert_with_source_location(                             \
        (condition), #condition, fmt::format(__VA_ARGS__), __FILE__, __LINE__)
