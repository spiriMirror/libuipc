#include <pyuipc/geometry/implicit_geometry_slot.h>
#include <uipc/geometry/implicit_geometry_slot.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyImplicitGeometrySlot::PyImplicitGeometrySlot(py::module& m)
{
    auto class_ImplicitGeometrySlot =
        py::class_<ImplicitGeometrySlot, GeometrySlot, S<ImplicitGeometrySlot>>(m, "ImplicitGeometrySlot");

    class_ImplicitGeometrySlot.def(
        "geometry",
        [&](ImplicitGeometrySlot& self) -> ImplicitGeometry&
        { return self.geometry(); },
        py::return_value_policy::reference_internal);
}
}  // namespace pyuipc::geometry
