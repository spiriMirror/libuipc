#include <pyuipc/constitution/affine_body_spherical_joint.h>
#include <uipc/constitution/affine_body_spherical_joint.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodySphericalJoint::PyAffineBodySphericalJoint(py::module& m)
{
    auto class_AffineBodySphericalJoint =
        py::class_<AffineBodySphericalJoint, InterAffineBodyConstitution>(
            m,
            "AffineBodySphericalJoint",
            R"(AffineBodySphericalJoint constitution for ball-and-socket joints between affine bodies.
Constrains only the translational degrees of freedom (anchor points coincide),
while allowing free relative rotation.
The anchor is specified as a position in body1 (right body)'s local frame.)");

    class_AffineBodySphericalJoint.def(py::init<const Json&>(),
                                       py::arg("config") =
                                           AffineBodySphericalJoint::default_config(),
                                       R"(Create an AffineBodySphericalJoint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodySphericalJoint.def_static("default_config",
                                              &AffineBodySphericalJoint::default_config,
                                              R"(Get the default AffineBodySphericalJoint configuration.
Returns:
    dict: Default configuration dictionary.)");

    // Simplified API: per-joint anchors + uniform strength
    class_AffineBodySphericalJoint.def(
        "apply_to",
        [](AffineBodySphericalJoint&    self,
           geometry::SimplicialComplex& sc,
           py::list                     l_geo_slots,
           py::list                     r_geo_slots,
           py::array_t<Float>           r_local_pos,
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

            self.apply_to(sc,
                          span{l_slots},
                          span{r_slots},
                          as_span_of<Vector3>(r_local_pos),
                          strength_ratio);
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("r_geo_slots"),
        py::arg("r_local_pos"),
        py::arg("strength_ratio") = Float{100.0},
        py::doc(R"(Create a spherical joint between two affine bodies (single-instance mode).
sc: An empty SimplicialComplex; vertices and edges are built internally.
l_geo_slots: List of left geometry slots for each joint.
r_geo_slots: List of right geometry slots for each joint.
r_local_pos: Array of [x, y, z] anchor positions in body1 (right body)'s local frame (one per joint).
strength_ratio: Stiffness = strength_ratio * (BodyMassA + BodyMassB) for all joints.)"));

    // Full API: explicit local anchor position on body1
    class_AffineBodySphericalJoint.def(
        "apply_to",
        [](AffineBodySphericalJoint&    self,
           geometry::SimplicialComplex& sc,
           py::list                     l_geo_slots,
           py::array_t<IndexT>          l_instance_ids,
           py::list                     r_geo_slots,
           py::array_t<IndexT>          r_instance_ids,
           py::array_t<Float>           r_local_pos,
           py::array_t<Float>           strength_ratios)
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

            self.apply_to(sc,
                          span{l_slots},
                          as_span<IndexT>(l_instance_ids),
                          span{r_slots},
                          as_span<IndexT>(r_instance_ids),
                          as_span_of<Vector3>(r_local_pos),
                          as_span<Float>(strength_ratios));
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("r_local_pos"),
        py::arg("strength_ratios"),
        py::doc(R"(Create a spherical joint between two affine bodies (multi-instance mode).
sc: An empty SimplicialComplex; vertices and edges are built internally.
l_geo_slots: List of left geometry slots for each joint.
l_instance_ids: List of instance IDs for left geometries.
r_geo_slots: List of right geometry slots for each joint.
r_instance_ids: List of instance IDs for right geometries.
r_local_pos: List of [x, y, z] anchor positions in body1 (right body)'s local frame.
strength_ratios: List of strength ratios for each joint (one per edge).)"));
}
}  // namespace pyuipc::constitution
