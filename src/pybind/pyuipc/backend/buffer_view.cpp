#include <pyuipc/backend/buffer_view.h>
#include <uipc/backend/buffer_view.h>
#include <fmt/format.h>
namespace pyuipc::backend
{
using namespace uipc::backend;
PyBufferView::PyBufferView(py::module& m)
{
    auto class_BufferView =
        py::class_<BufferView>(m, "BufferView",
                               R"(BufferView class representing a view into a backend buffer.)")
            .def(py::init<>(),
                 R"(Create an empty buffer view.)")
            .def(py::init<HandleT, SizeT, SizeT, SizeT, SizeT, std::string_view>(),
                 py::arg("handle"),
                 py::arg("element_offset"),
                 py::arg("element_count"),
                 py::arg("element_size"),
                 py::arg("element_stride"),
                 py::arg("backend_name"),
                 R"(Create a buffer view with specified parameters.
Args:
    handle: Backend handle to the buffer.
    element_offset: Offset in elements from the start of the buffer.
    element_count: Number of elements in the view.
    element_size: Size of each element in bytes.
    element_stride: Stride between elements in bytes.
    backend_name: Name of the backend.)")
            .def("handle", &BufferView::handle,
                 R"(Get the backend handle.
Returns:
    int: Backend handle value.)")
            .def("offset", &BufferView::offset,
                 R"(Get the element offset.
Returns:
    int: Element offset.)")
            .def("size", &BufferView::size,
                 R"(Get the number of elements.
Returns:
    int: Number of elements.)")
            .def("element_size", &BufferView::element_size,
                 R"(Get the element size in bytes.
Returns:
    int: Element size in bytes.)")
            .def("element_stride", &BufferView::element_stride,
                 R"(Get the element stride in bytes.
Returns:
    int: Element stride in bytes.)")
            .def("size_in_bytes", &BufferView::size_in_bytes,
                 R"(Get the total size in bytes.
Returns:
    int: Total size in bytes.)")
            .def("backend", &BufferView::backend,
                 R"(Get the backend name.
Returns:
    str: Backend name.)")
            .def("__bool__", &BufferView::operator bool,
                 R"(Check if the buffer view is valid.
Returns:
    bool: True if valid, False otherwise.)")
            .def("subview", &BufferView::subview, py::arg("offset"), py::arg("element_count"),
                 R"(Create a subview of this buffer view.
Args:
    offset: Element offset from the start of this view.
    element_count: Number of elements in the subview.
Returns:
    BufferView: New buffer view representing the subview.)");

    class_BufferView.def("__repr__",
                         [](const BufferView& bv)
                         {
                             return fmt::format(
                                 "BufferView(handle={}, offset={}, size={}, element_size={}, element_stride={}, "
                                 "backend='{}')",
                                 bv.handle(),
                                 bv.offset(),
                                 bv.size(),
                                 bv.element_size(),
                                 bv.element_stride(),
                                 bv.backend());
                         },
                         R"(String representation of the buffer view.
Returns:
    str: Formatted string representation.)");
}
}  // namespace pyuipc::backend
