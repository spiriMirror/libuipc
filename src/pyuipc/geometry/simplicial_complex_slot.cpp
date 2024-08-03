#include <pyuipc/geometry/simplicial_complex_slot.h>
#include <uipc/geometry/simplicial_complex_slot.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;
PySimplicialComplexSlot::PySimplicialComplexSlot(py::module& m)

{
    auto class_SimplicialComplexSlot =
        py::class_<SimplicialComplexSlot, GeometrySlot, S<SimplicialComplexSlot>>(m, "SimplicialComplexSlot");

    class_SimplicialComplexSlot.def(
        "geometry",
        [](SimplicialComplexSlot& self) -> SimplicialComplex&
        { return self.geometry(); },
        py::return_value_policy::reference_internal);
}
}  // namespace pyuipc::geometry
