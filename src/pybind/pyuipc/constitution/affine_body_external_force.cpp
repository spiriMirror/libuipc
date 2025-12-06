#include <pyuipc/constitution/affine_body_external_force.h>
#include <uipc/constitution/affine_body_external_force.h>
#include <pyuipc/as_numpy.h>
#include <pybind11/eigen.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

static void bind_affine_body_external_force(py::module& m)
{
    using namespace uipc;

    auto class_affine_body_external_force =
        py::class_<AffineBodyExternalForce, IConstitution>(m, "AffineBodyExternalForce");

    class_affine_body_external_force.def(py::init<>())
        .def(py::init<const Json&>(), py::arg("config"))
        .def("apply_to",
             py::overload_cast<geometry::SimplicialComplex&, const Vector12&>(
                 &AffineBodyExternalForce::apply_to),
             py::arg("sc"),
             py::arg("force"),
             R"(Apply external force (12D generalized force) to affine body instances.

             Args:
                 sc: SimplicialComplex representing affine body geometry
                 force: 12D generalized force vector [fx, fy, fz, f_a11, f_a12, ..., f_a33]
                        where f is 3D translational force and f_a is 9D affine force
             )")
        .def("apply_to",
             py::overload_cast<geometry::SimplicialComplex&, const Vector3&>(
                 &AffineBodyExternalForce::apply_to),
             py::arg("sc"),
             py::arg("force"),
             R"(Apply external translational force to affine body instances.

             Args:
                 sc: SimplicialComplex representing affine body geometry
                 force: 3D translational force vector (affine force = 0)
             )")
        .def_static("default_config", &AffineBodyExternalForce::default_config);
}

PyAffineBodyExternalForce::PyAffineBodyExternalForce(py::module& m)
{
    bind_affine_body_external_force(m);
}
}  // namespace pyuipc::constitution
