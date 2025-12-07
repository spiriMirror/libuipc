#include <pyuipc/geometry/simplicial_complex_slot.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PySimplicialComplexSlot::PySimplicialComplexSlot(py::module& m)

{
    auto class_SimplicialComplexSlot =
        py::class_<SimplicialComplexSlot, GeometrySlot, S<SimplicialComplexSlot>>(m, "SimplicialComplexSlot",
                                                                                   R"(SimplicialComplexSlot class representing a slot containing a simplicial complex.)");

    class_SimplicialComplexSlot.def(
        "geometry",
        [](SimplicialComplexSlot& self) -> SimplicialComplex&
        { return self.geometry(); },
        py::return_value_policy::reference_internal,
        R"(Get the simplicial complex in this slot.
Returns:
    SimplicialComplex: Reference to the simplicial complex.)");
}
}  // namespace pyuipc::geometry
