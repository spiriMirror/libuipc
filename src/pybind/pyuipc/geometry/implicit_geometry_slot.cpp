#include <pyuipc/geometry/implicit_geometry_slot.h>
#include <uipc/geometry/implicit_geometry_slot.h>
namespace pyuipc::geometry
{
using namespace uipc::geometry;
PyImplicitGeometrySlot::PyImplicitGeometrySlot(py::module_& m)
{
    auto class_ImplicitGeometrySlot =
        py::class_<ImplicitGeometrySlot, GeometrySlot>(
            m, "ImplicitGeometrySlot", R"(ImplicitGeometrySlot class representing a slot containing an implicit geometry.)");

    class_ImplicitGeometrySlot.def(
        "geometry",
        [&](ImplicitGeometrySlot& self) -> ImplicitGeometry&
        { return self.geometry(); },
        py::rv_policy::reference_internal,
        R"(Get the implicit geometry in this slot.
Returns:
    ImplicitGeometry: Reference to the implicit geometry.)");
}
}  // namespace pyuipc::geometry
