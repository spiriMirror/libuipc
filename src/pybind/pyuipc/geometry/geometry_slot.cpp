#include <pyuipc/geometry/geometry_slot.h>
#include <uipc/geometry/geometry_slot.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyGeometrySlot::PyGeometrySlot(py::module& m)
{
    auto class_GeometrySlot = py::class_<GeometrySlot, S<GeometrySlot>>(m, "GeometrySlot",
                                                                          R"(GeometrySlot class representing a slot containing a geometry.)");
    class_GeometrySlot.def("id", [](GeometrySlot& self) { return self.id(); },
                          R"(Get the geometry slot ID.
Returns:
    int: Geometry slot ID.)");
    class_GeometrySlot.def(
        "geometry",
        [](GeometrySlot& self) -> Geometry& { return self.geometry(); },
        py::return_value_policy::reference_internal,
        R"(Get the geometry in this slot.
Returns:
    Geometry: Reference to the geometry.)");
}
}  // namespace pyuipc::geometry
