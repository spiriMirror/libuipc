#include <pyuipc/backend/buffer_view.h>
#include <uipc/backend/buffer_view.h>
#include <fmt/format.h>
namespace pyuipc::backend
{
using namespace uipc::backend;
PyBufferView::PyBufferView(py::module& m)
{
    auto class_BufferView =
        py::class_<BufferView>(m, "BufferView")
            .def(py::init<>())
            .def(py::init<HandleT, SizeT, SizeT, SizeT, SizeT, std::string_view>(),
                 py::arg("handle"),
                 py::arg("element_offset"),
                 py::arg("element_count"),
                 py::arg("element_size"),
                 py::arg("element_stride"),
                 py::arg("backend_name"))
            .def("handle", &BufferView::handle)
            .def("offset", &BufferView::offset)
            .def("size", &BufferView::size)
            .def("element_size", &BufferView::element_size)
            .def("element_stride", &BufferView::element_stride)
            .def("size_in_bytes", &BufferView::size_in_bytes)
            .def("backend", &BufferView::backend)
            .def("__bool__", &BufferView::operator bool)
            .def("subview", &BufferView::subview, py::arg("offset"), py::arg("element_count"));

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
                         });
}
}  // namespace pyuipc::backend
