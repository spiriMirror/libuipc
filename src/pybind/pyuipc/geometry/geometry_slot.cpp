#include <pyuipc/geometry/geometry_slot.h>
#include <uipc/geometry/geometry_slot.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyGeometrySlot::PyGeometrySlot(py::module& m)
{
    auto class_GeometrySlot = py::class_<GeometrySlot, S<GeometrySlot>>(m, "GeometrySlot");
    class_GeometrySlot.def("id", [](GeometrySlot& self) { return self.id(); });
    class_GeometrySlot.def(
        "geometry",
        [](GeometrySlot& self) -> Geometry& { return self.geometry(); },
        py::return_value_policy::reference_internal);
}
}  // namespace pyuipc::geometry
