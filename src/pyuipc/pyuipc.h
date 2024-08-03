#pragma once
#include <pybind11/pybind11.h>
#include <functional>
#include <uipc/common/type_define.h>

namespace pyuipc
{
using namespace uipc;
namespace py = pybind11;

namespace detail
{
    std::string string_with_source_location(std::string_view msg,
                                            std::string_view path,
                                            std::size_t      line);
}
}  // namespace pyuipc

#define PYUIPC_MSG(...)                                                        \
    ::pyuipc::detail::string_with_source_location(fmt::format(__VA_ARGS__), __FILE__, __LINE__)
