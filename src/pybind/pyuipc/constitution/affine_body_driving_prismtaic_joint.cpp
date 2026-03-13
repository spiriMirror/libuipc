#include <pyuipc/constitution/affine_body_driving_prismatic_joint.h>
#include <uipc/constitution/affine_body_driving_prismatic_joint.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyDrivingPrismaticJoint::PyAffineBodyDrivingPrismaticJoint(py::module& m)
{
    auto class_AffineBodyDrivingPrismaticJoint =
        py::class_<AffineBodyDrivingPrismaticJoint, IConstitution>(
            m,
            "AffineBodyDrivingPrismaticJoint",
            R"(AffineBodyDrivingPrismaticJoint constitution for driving prismatic (sliding) joints between affine bodies.)");

    class_AffineBodyDrivingPrismaticJoint.def(
        py::init<const Json&>(),
        py::arg("config") = AffineBodyDrivingPrismaticJoint::default_config(),
        R"(Create an AffineBodyDrivingPrismaticJoint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyDrivingPrismaticJoint.def_static("default_config",
                                                     &AffineBodyDrivingPrismaticJoint::default_config,
                                                     R"(Get the default AffineBodyDrivingPrismaticJoint configuration.
Returns:
    dict: Default configuration dictionary.)");

    // Single-instance mode
    class_AffineBodyDrivingPrismaticJoint.def(
        "apply_to",
        [](AffineBodyDrivingPrismaticJoint& self, geometry::SimplicialComplex& edges, Float strength_ratio)
        { self.apply_to(edges, strength_ratio); },
        py::arg("sc"),
        py::arg("strength_ratio") = Float{100.0},
        py::doc(R"(Create joint between two affine bodies (single-instance mode).
edges: Every edge in the simplicial complex is treated as a joint axis.
strength_ratio: The strength ratio of the joint constraint applied to all joints (default: 100).)"));

    // Multi-instance mode
    class_AffineBodyDrivingPrismaticJoint.def(
        "apply_to",
        [](AffineBodyDrivingPrismaticJoint& self, geometry::SimplicialComplex& edges, py::list strength_ratios)
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
        py::doc(R"(Create joint between two affine bodies (multi-instance mode).
edges: Every edge in the simplicial complex is treated as a joint axis.
strength_ratios: List of strength ratios for each joint (one per edge).)"));
}
}  // namespace pyuipc::constitution
