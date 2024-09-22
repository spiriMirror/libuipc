#include <pyuipc/geometry/simplex_slot.h>
#include <uipc/geometry/simplex_slot.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PySimplexSlot::PySimplexSlot(py::module& m)
{
    py::class_<ISimplexSlot>(m, "ISimplexSlot")
        .def("size", &ISimplexSlot::size)
        .def("is_shared", &ISimplexSlot::is_shared)
        .def("reorder",
             [](ISimplexSlot& self, py::array_t<SizeT> O)
             { self.reorder(as_span(O)); });

    py::class_<VertexSlot, ISimplexSlot>(m, "VertexSlot")
        .def("view",
             [](VertexSlot& self) { as_numpy(self.view(), py::cast(self)); });

    m.def("view", [](VertexSlot& self) { as_numpy(view(self), py::cast(self)); });


    py::class_<EdgeSlot, ISimplexSlot>(m, "EdgeSlot")
        .def("view", [](EdgeSlot& self) { as_numpy(self.view(), py::cast(self)); });

    m.def("view", [](EdgeSlot& self) { as_numpy(view(self), py::cast(self)); });

    py::class_<TriangleSlot, ISimplexSlot>(m, "TriangleSlot")
        .def("view", [](TriangleSlot& self) { as_numpy(self.view(), py::cast(self)); });

    m.def("view", [](TriangleSlot& self) { as_numpy(view(self), py::cast(self)); });

    py::class_<TetrahedronSlot, ISimplexSlot>(m, "TetrahedronSlot")
        .def("view", [](TetrahedronSlot& self) { as_numpy(self.view(), py::cast(self)); });

    m.def("view", [](TetrahedronSlot& self) { as_numpy(view(self), py::cast(self)); });
}
}  // namespace pyuipc::geometry
