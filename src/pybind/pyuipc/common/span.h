#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc
{
template <typename T>
auto def_span(py::handle& h, const char* name)
{
    auto class_Span = py::class_<span<T>>(h, name);
    class_Span.def("__len__", &span<T>::size);
    class_Span.def("__getitem__", [](span<T>& s, size_t i) { return s[i]; });
    class_Span.def(
        "__iter__",
        [](span<T>& s) { return py::make_iterator(s.begin(), s.end()); },
        py::keep_alive<0, 1>());
    if constexpr(!std::is_const_v<T>)
    {
        class_Span.def("__setitem__", [](span<T>& s, size_t i, T v) { s[i] = v; });
    }
    return class_Span;
}
}  // namespace pyuipc