#include <pyuipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/constitution.h>
#include <pyuipc/common/json.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyRevoluteJoint::PyAffineBodyRevoluteJoint(py::module& m)
{
    auto class_AffineBodyRevoluteJoint =
        py::class_<AffineBodyRevoluteJoint, InterAffineBodyConstitution>(m, "AffineBodyRevoluteJoint",
                                                                          R"(AffineBodyRevoluteJoint constitution for revolute (hinge) joints between affine bodies.)");

    class_AffineBodyRevoluteJoint.def(py::init<const Json&>(),
                                      py::arg("config") =
                                          AffineBodyRevoluteJoint::default_config(),
                                      R"(Create an AffineBodyRevoluteJoint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyRevoluteJoint.def_static("default_config",
                                             &AffineBodyRevoluteJoint::default_config,
                                             R"(Get the default AffineBodyRevoluteJoint configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_AffineBodyRevoluteJoint.def(
        "apply_to",
        [](AffineBodyRevoluteJoint& self, geometry::SimplicialComplex& edges, py::list geo_slot_tuples, Float strength_ratio)
        {
            vector<AffineBodyRevoluteJoint::SlotTuple> geo_slots;
            geo_slots.reserve(geo_slot_tuples.size());
            for(auto item : geo_slot_tuples)
            {
                if(!py::isinstance<py::tuple>(item))
                {
                    throw std::invalid_argument("geo_slot_tuples must be a list of tuples");
                }
                geo_slots.push_back(py::cast<AffineBodyRevoluteJoint::SlotTuple>(item));
            }
            self.apply_to(edges, geo_slots, strength_ratio);
        },
        py::arg("sc"),
        py::arg("geo_slot_tuples"),
        py::arg("strength_ratio") = Float{100.0},
        py::doc(R"(Create joint between two affine bodies.
sc: Every edge in the simplicial complex is treated as a joint axis.
geo_slot_tuples: A list of tuples, each containing two SimplicialComplexSlot objects, telling who are linked by the joint.
strength_ratio: Stiffness = strength_ratio * (BodyMassA + BodyMassB))"));
}
}  // namespace pyuipc::constitution
