#include <pyuipc/constitution/finite_element_external_force.h>
#include <uipc/constitution/finite_element_external_force.h>
#include <pybind11/eigen.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyFiniteElementExternalForce::PyFiniteElementExternalForce(py::module& m)
{
    auto class_FiniteElementExternalForce =
        py::class_<FiniteElementExternalForce, IConstitution>(m, "FiniteElementExternalForce");

    class_FiniteElementExternalForce
        .def(py::init<const Json&>(),
             py::arg("config") = FiniteElementExternalForce::default_config())
        .def("apply_to",
             &FiniteElementExternalForce::apply_to,
             py::arg("sc"),
             py::arg("force"),
             R"(Apply external force (3D) to finite element vertices.

             Args:
                 sc: SimplicialComplex representing finite element geometry
                 force: 3D force vector applied uniformly to all vertices
             )")
        .def_static("default_config",
                    &FiniteElementExternalForce::default_config);
}
}  // namespace pyuipc::constitution
