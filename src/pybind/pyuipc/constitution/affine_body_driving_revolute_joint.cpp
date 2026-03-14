#include <pyuipc/constitution/affine_body_driving_revolute_joint.h>
#include <uipc/constitution/affine_body_driving_revolute_joint.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyDrivingRevoluteJoint::PyAffineBodyDrivingRevoluteJoint(py::module_& m)
{
    auto class_AffineBodyDrivingRevoluteJoint =
        py::class_<AffineBodyDrivingRevoluteJoint, IConstitution>(
            m,
            "AffineBodyDrivingRevoluteJoint",
            R"(AffineBodyDrivingRevoluteJoint constitution for driving revolute (hinge) joints between affine bodies.)");

    class_AffineBodyDrivingRevoluteJoint.def(
        py::init<const Json&>(),
        py::arg("config") = AffineBodyDrivingRevoluteJoint::default_config(),
        R"(Create an AffineBodyDrivingRevoluteJoint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyDrivingRevoluteJoint.def_static("default_config",
                                                    &AffineBodyDrivingRevoluteJoint::default_config,
                                                    R"(Get the default AffineBodyDrivingRevoluteJoint configuration.
Returns:
    dict: Default configuration dictionary.)");

    // Single-instance mode
    class_AffineBodyDrivingRevoluteJoint.def(
        "apply_to",
        [](AffineBodyDrivingRevoluteJoint& self, geometry::SimplicialComplex& edges, Float strength_ratio)
        { self.apply_to(edges, strength_ratio); },
        py::arg("sc"),
        py::arg("strength_ratio") = Float{100.0},
        R"(Create joint between two affine bodies (single-instance mode).
edges: Every edge in the simplicial complex is treated as a joint axis.
strength_ratio: The strength ratio of the joint constraint applied to all joints (default: 100).)");


    // New API - Multi-instance mode
    class_AffineBodyDrivingRevoluteJoint.def(
        "apply_to",
        [](AffineBodyDrivingRevoluteJoint& self, geometry::SimplicialComplex& edges, py::list strength_ratios)
        {
            vector<Float> strength_ratios_list;

            for(auto item : strength_ratios)
            {
                strength_ratios_list.push_back(py::cast<Float>(item));
            }
            self.apply_to(edges, span{strength_ratios_list});
        },
        py::arg("sc"),
        py::arg("strength_ratios"),
        R"(Create joint between two affine bodies (multi-instance mode).
edges: Every edge in the simplicial complex is treated as a joint axis.
strength_ratios: List of strength ratios for each joint (one per edge).)");
}
}  // namespace pyuipc::constitution
