#include <pyuipc/backend/buffer_view.h>
#include <uipc/backend/buffer_view.h>
namespace pyuipc::backend
{
using namespace uipc::backend;
PyBufferView::PyBufferView(py::module& m)
{
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
}
}  // namespace pyuipc::backend
