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
             py::arg("enable") = true,
             R"(Apply external force (3D) to finite element vertices.

             Sets external_force/is_constrained on all vertices. The shared
             is_constrained flag for soft constraints is unchanged. Use the
             dedicated attribute in the Animator for selective or delayed forces.
             Reapplying resets the force and its enable mask on all vertices.

             Args:
                 sc: SimplicialComplex representing finite element geometry
                 force: 3D force vector applied uniformly to all vertices
                 enable: Enable external force (default True); False stores a disabled force
             )")
        .def_static("default_config", &FiniteElementExternalForce::default_config);
}
}  // namespace pyuipc::constitution
