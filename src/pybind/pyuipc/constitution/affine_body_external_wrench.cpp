#include <pyuipc/pyuipc.h>
#include <uipc/constitution/affine_body_external_wrench.h>
#include <pyuipc/as_numpy.h>
#include <pybind11/eigen.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

void bind_affine_body_external_wrench(py::module& m)
{
    using namespace uipc;

    auto class_affine_body_external_wrench =
        py::class_<AffineBodyExternalWrench, IConstitution>(m, "AffineBodyExternalWrench");

    class_affine_body_external_wrench.def(py::init<>())
        .def(py::init<const Json&>(), py::arg("config"))
        .def("apply_to",
             py::overload_cast<geometry::SimplicialComplex&, const Vector12&>(
                 &AffineBodyExternalWrench::apply_to),
             py::arg("sc"),
             py::arg("wrench"),
             R"(Apply external wrench (generalized force) to affine body instances.

             Args:
                 sc: SimplicialComplex representing affine body geometry
                 wrench: 12D generalized force vector [fx, fy, fz, dS11/dt, dS12/dt, ..., dS33/dt]
                         where f is 3D translational force and dS/dt is shape velocity derivative (flattened 3x3)
             )")
        .def("apply_to",
             py::overload_cast<geometry::SimplicialComplex&, const Vector3&>(
                 &AffineBodyExternalWrench::apply_to),
             py::arg("sc"),
             py::arg("force"),
             R"(Apply external translational force to affine body instances.

             Args:
                 sc: SimplicialComplex representing affine body geometry
                 force: 3D translational force vector (shape velocity derivative = 0)
             )")
        .def_static("default_config", &AffineBodyExternalWrench::default_config);
}
}  // namespace pyuipc::constitution
