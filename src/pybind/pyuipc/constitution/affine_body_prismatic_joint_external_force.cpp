#include <pyuipc/constitution/affine_body_prismatic_joint_external_force.h>
#include <uipc/constitution/affine_body_prismatic_joint_external_force.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyPrismaticJointExternalForce::PyAffineBodyPrismaticJointExternalForce(py::module& m)
{
    auto class_AffineBodyPrismaticJointExternalForce =
        py::class_<AffineBodyPrismaticJointExternalForce, Constraint>(
            m,
            "AffineBodyPrismaticJointExternalForce",
            R"(Applies external force along the prismatic joint axis between two connected affine bodies.)");

    class_AffineBodyPrismaticJointExternalForce.def(
        py::init<const Json&>(),
        py::arg("config") = AffineBodyPrismaticJointExternalForce::default_config(),
        R"(Create an AffineBodyPrismaticJointExternalForce.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyPrismaticJointExternalForce.def_static(
        "default_config",
        &AffineBodyPrismaticJointExternalForce::default_config,
        R"(Get the default configuration.
Returns:
    dict: Default configuration dictionary.)");

    // Uniform force for all joints
    class_AffineBodyPrismaticJointExternalForce.def(
        "apply_to",
        [](AffineBodyPrismaticJointExternalForce& self, geometry::SimplicialComplex& sc, Float force)
        { self.apply_to(sc, force); },
        py::arg("sc"),
        py::arg("force") = Float{0},
        py::doc(R"(Apply uniform external force along prismatic joint axis to all joints.
sc: Simplicial complex with edges representing the joints (must have AffineBodyPrismaticJoint applied).
force: Scalar force value applied to all joints (default: 0).)"));

    // Per-joint forces
    class_AffineBodyPrismaticJointExternalForce.def(
        "apply_to",
        [](AffineBodyPrismaticJointExternalForce& self,
           geometry::SimplicialComplex&           sc,
           py::array_t<Float>                     forces)
        { self.apply_to(sc, as_span<Float>(forces)); },
        py::arg("sc"),
        py::arg("forces"),
        py::doc(R"(Apply per-joint external forces along prismatic joint axis.
sc: Simplicial complex with edges representing the joints (must have AffineBodyPrismaticJoint applied).
forces: List of scalar force values (one per edge).)"));
}
}  // namespace pyuipc::constitution
