#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc
{
template <typename T>
auto def_span(py::handle& h, const char* name)
{
    auto class_Span = py::class_<span<T>>(h, name,
                                          R"(Span class for viewing contiguous sequences of data.)");
    class_Span.def("__len__", &span<T>::size,
                  R"(Get the size of the span.
Returns:
    int: Number of elements.)");
    class_Span.def("__getitem__", [](span<T>& s, size_t i) { return s[i]; },
                  py::arg("index"),
                  R"(Get an element by index.
Args:
    index: Element index.
Returns:
    Element value.)");
    class_Span.def(
        "__iter__",
        [](span<T>& s) { return py::make_iterator(s.begin(), s.end()); },
        py::keep_alive<0, 1>(),
        R"(Create an iterator over the span.
Returns:
    iterator: Iterator over span elements.)");
    if constexpr(!std::is_const_v<T>)
    {
        class_Span.def("__setitem__", [](span<T>& s, size_t i, T v) { s[i] = v; },
                      py::arg("index"),
                      py::arg("value"),
                      R"(Set an element by index.
Args:
    index: Element index.
    value: Value to set.)");
    }
    return class_Span;
}
}  // namespace pyuipc