#include <pyuipc/constitution/affine_body_revolute_joint_external_force.h>
#include <uipc/constitution/affine_body_revolute_joint_external_force.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyRevoluteJointExternalForce::PyAffineBodyRevoluteJointExternalForce(py::module& m)
{
    auto class_AffineBodyRevoluteJointExternalForce =
        py::class_<AffineBodyRevoluteJointExternalForce, Constraint>(
            m,
            "AffineBodyRevoluteJointExternalForce",
            R"(Applies external torque around the revolute joint axis between two connected affine bodies.)");

    class_AffineBodyRevoluteJointExternalForce.def(
        py::init<const Json&>(),
        py::arg("config") = AffineBodyRevoluteJointExternalForce::default_config(),
        R"(Create an AffineBodyRevoluteJointExternalForce.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyRevoluteJointExternalForce.def_static(
        "default_config",
        &AffineBodyRevoluteJointExternalForce::default_config,
        R"(Get the default configuration.
Returns:
    dict: Default configuration dictionary.)");

    // Uniform torque for all joints
    class_AffineBodyRevoluteJointExternalForce.def(
        "apply_to",
        [](AffineBodyRevoluteJointExternalForce& self,
           geometry::SimplicialComplex&               sc,
           Float                                      torque)
        { self.apply_to(sc, torque); },
        py::arg("sc"),
        py::arg("torque") = Float{0},
        py::doc(R"(Apply uniform external torque around revolute joint axis to all joints.
sc: Simplicial complex with edges representing the joints (must have AffineBodyRevoluteJoint applied).
torque: Scalar torque value applied to all joints (default: 0).)"));

    // Per-joint torques
    class_AffineBodyRevoluteJointExternalForce.def(
        "apply_to",
        [](AffineBodyRevoluteJointExternalForce& self,
           geometry::SimplicialComplex&               sc,
           py::list                                   torques)
        {
            vector<Float> torques_list;
            for(auto item : torques)
            {
                torques_list.push_back(py::cast<Float>(item));
            }
            self.apply_to(sc, span{torques_list});
        },
        py::arg("sc"),
        py::arg("torques"),
        py::doc(R"(Apply per-joint external torques around revolute joint axis.
sc: Simplicial complex with edges representing the joints (must have AffineBodyRevoluteJoint applied).
torques: List of scalar torque values (one per edge).)"));
}
}  // namespace pyuipc::constitution
