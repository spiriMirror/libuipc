#include <pyuipc/constitution/affine_body_prismatic_joint_external_force.h>
#include <uipc/constitution/affine_body_prismatic_joint_external_force.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyPrismaticJointExternalForce::PyAffineBodyPrismaticJointExternalForce(py::module& m)
{
    auto class_AffineBodyPrismaticJointExternalBodyForce =
        py::class_<AffineBodyPrismaticJointExternalBodyForce, Constraint>(
            m,
            "AffineBodyPrismaticJointExternalBodyForce",
            R"(Applies external force along the prismatic joint axis between two connected affine bodies.)");

    class_AffineBodyPrismaticJointExternalBodyForce.def(
        py::init<const Json&>(),
        py::arg("config") = AffineBodyPrismaticJointExternalBodyForce::default_config(),
        R"(Create an AffineBodyPrismaticJointExternalBodyForce.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyPrismaticJointExternalBodyForce.def_static(
        "default_config",
        &AffineBodyPrismaticJointExternalBodyForce::default_config,
        R"(Get the default configuration.
Returns:
    dict: Default configuration dictionary.)");

    // Uniform force for all joints
    class_AffineBodyPrismaticJointExternalBodyForce.def(
        "apply_to",
        [](AffineBodyPrismaticJointExternalBodyForce& self,
           geometry::SimplicialComplex&                sc,
           Float                                       force)
        { self.apply_to(sc, force); },
        py::arg("sc"),
        py::arg("force") = Float{0},
        py::doc(R"(Apply uniform external force along prismatic joint axis to all joints.
sc: Simplicial complex with edges representing the joints (must have AffineBodyPrismaticJoint applied).
force: Scalar force value applied to all joints (default: 0).)"));

    // Per-joint forces
    class_AffineBodyPrismaticJointExternalBodyForce.def(
        "apply_to",
        [](AffineBodyPrismaticJointExternalBodyForce& self,
           geometry::SimplicialComplex&                sc,
           py::list                                    forces)
        {
            vector<Float> forces_list;
            for(auto item : forces)
            {
                forces_list.push_back(py::cast<Float>(item));
            }
            self.apply_to(sc, span{forces_list});
        },
        py::arg("sc"),
        py::arg("forces"),
        py::doc(R"(Apply per-joint external forces along prismatic joint axis.
sc: Simplicial complex with edges representing the joints (must have AffineBodyPrismaticJoint applied).
forces: List of scalar force values (one per edge).)"));
}
}  // namespace pyuipc::constitution
