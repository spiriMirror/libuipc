#pragma once
#include <uipc/common/span.h>

namespace pyuipc
{
template <typename T>
py::array_t<T> as_numpy(span<T> s)
{
    return py::array_t<T>(s.size(), s.data());
}
}  // namespace pyuipc