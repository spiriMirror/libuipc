#include <pyuipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/affine_body_revolute_joint.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyRevoluteJoint::PyAffineBodyRevoluteJoint(py::module& m)
{
    auto class_AffineBodyRevoluteJoint =
        py::class_<AffineBodyRevoluteJoint, InterAffineBodyConstitution>(
            m, "AffineBodyRevoluteJoint", R"(AffineBodyRevoluteJoint constitution for revolute (hinge) joints between affine bodies.)");

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

    //  Single instance mode
    class_AffineBodyRevoluteJoint.def(
        "apply_to",
        [](AffineBodyRevoluteJoint&     self,
           geometry::SimplicialComplex& edges,
           py::list                     l_geo_slots,
           py::list                     r_geo_slots,
           Float                        strength_ratio)
        {
            vector<S<geometry::SimplicialComplexSlot>> l_slots;
            vector<S<geometry::SimplicialComplexSlot>> r_slots;
            l_slots.reserve(l_geo_slots.size());
            r_slots.reserve(r_geo_slots.size());

            for(auto item : l_geo_slots)
            {
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            }
            for(auto item : r_geo_slots)
            {
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            }
            self.apply_to(edges, span{l_slots}, span{r_slots}, strength_ratio);
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("r_geo_slots"),
        py::arg("strength_ratio") = Float{100.0},
        py::doc(R"(Create joint between two affine bodies (single-instance mode).
sc: Every edge in the simplicial complex is treated as a joint axis.
l_geo_slots: List of left geometry slots for each joint.
r_geo_slots: List of right geometry slots for each joint.
strength_ratio: Stiffness = strength_ratio * (BodyMassA + BodyMassB) for all joints.)"));

    // Multi-instance mode
    class_AffineBodyRevoluteJoint.def(
        "apply_to",
        [](AffineBodyRevoluteJoint&     self,
           geometry::SimplicialComplex& edges,
           py::list                     l_geo_slots,
           py::array_t<IndexT>          l_instance_id,
           py::list                     r_geo_slots,
           py::array_t<IndexT>          r_instance_id,
           py::array_t<Float>           strength_ratio)
        {
            vector<S<geometry::SimplicialComplexSlot>> l_slots;
            vector<S<geometry::SimplicialComplexSlot>> r_slots;

            l_slots.reserve(l_geo_slots.size());
            r_slots.reserve(r_geo_slots.size());

            for(auto item : l_geo_slots)
            {
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            }
            for(auto item : r_geo_slots)
            {
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            }

            self.apply_to(edges,
                          span{l_slots},
                          as_span<IndexT>(l_instance_id),
                          span{r_slots},
                          as_span<IndexT>(r_instance_id),
                          as_span<Float>(strength_ratio));
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_id"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_id"),
        py::arg("strength_ratio"),
        py::doc(R"(Create joint between two affine bodies (multi-instance mode).
sc: Every edge in the simplicial complex is treated as a joint axis.
l_geo_slots: List of left geometry slots for each joint.
l_instance_id: List of instance IDs for left geometries.
r_geo_slots: List of right geometry slots for each joint.
r_instance_id: List of instance IDs for right geometries.
strength_ratio: List of strength ratios for each joint (one per edge).)"));
}
}  // namespace pyuipc::constitution
