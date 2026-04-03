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

    // World create_geometry
    class_AffineBodyRevoluteJoint.def(
        "create_geometry",
        [](AffineBodyRevoluteJoint& self,
           py::array_t<Float>       position0s,
           py::array_t<Float>       position1s,
           py::list                 l_geo_slots,
           py::array_t<IndexT>      l_instance_ids,
           py::list                 r_geo_slots,
           py::array_t<IndexT>      r_instance_ids,
           py::array_t<Float>       strength_ratios)
        {
            vector<S<geometry::SimplicialComplexSlot>> l_slots;
            vector<S<geometry::SimplicialComplexSlot>> r_slots;
            l_slots.reserve(l_geo_slots.size());
            r_slots.reserve(r_geo_slots.size());
            for(auto item : l_geo_slots)
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            for(auto item : r_geo_slots)
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));

            return self.create_geometry(as_span_of<Vector3>(position0s),
                                        as_span_of<Vector3>(position1s),
                                        span{l_slots},
                                        as_span<IndexT>(l_instance_ids),
                                        span{r_slots},
                                        as_span<IndexT>(r_instance_ids),
                                        as_span<Float>(strength_ratios));
        },
        py::arg("position0s"),
        py::arg("position1s"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("strength_ratios"),
        R"(Create revolute joint geometry using world-space axis endpoints.

Args:
    position0s: World positions of the first axis endpoint for each joint (Nx3 array).
    position1s: World positions of the second axis endpoint for each joint (Nx3 array).
    l_geo_slots: Left geometry slots for each joint.
    l_instance_ids: Instance IDs for left geometries.
    r_geo_slots: Right geometry slots for each joint.
    r_instance_ids: Instance IDs for right geometries.
    strength_ratios: Strength ratio for each joint.

Returns:
    SimplicialComplex: The joint geometry mesh.)");

    // Local create_geometry
    class_AffineBodyRevoluteJoint.def(
        "create_geometry",
        [](AffineBodyRevoluteJoint& self,
           py::array_t<Float>       l_position0,
           py::array_t<Float>       l_position1,
           py::array_t<Float>       r_position0,
           py::array_t<Float>       r_position1,
           py::list                 l_geo_slots,
           py::array_t<IndexT>      l_instance_ids,
           py::list                 r_geo_slots,
           py::array_t<IndexT>      r_instance_ids,
           py::array_t<Float>       strength_ratios)
        {
            vector<S<geometry::SimplicialComplexSlot>> l_slots;
            vector<S<geometry::SimplicialComplexSlot>> r_slots;
            l_slots.reserve(l_geo_slots.size());
            r_slots.reserve(r_geo_slots.size());
            for(auto item : l_geo_slots)
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            for(auto item : r_geo_slots)
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));

            return self.create_geometry(as_span_of<Vector3>(l_position0),
                                        as_span_of<Vector3>(l_position1),
                                        as_span_of<Vector3>(r_position0),
                                        as_span_of<Vector3>(r_position1),
                                        span{l_slots},
                                        as_span<IndexT>(l_instance_ids),
                                        span{r_slots},
                                        as_span<IndexT>(r_instance_ids),
                                        as_span<Float>(strength_ratios));
        },
        py::arg("l_position0"),
        py::arg("l_position1"),
        py::arg("r_position0"),
        py::arg("r_position1"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("strength_ratios"),
        R"(Create revolute joint geometry using local-space axis endpoints.

Args:
    l_position0: Local positions of the first axis endpoint on the left body (Nx3 array).
    l_position1: Local positions of the second axis endpoint on the left body (Nx3 array).
    r_position0: Local positions of the first axis endpoint on the right body (Nx3 array).
    r_position1: Local positions of the second axis endpoint on the right body (Nx3 array).
    l_geo_slots: Left geometry slots for each joint.
    l_instance_ids: Instance IDs for left geometries.
    r_geo_slots: Right geometry slots for each joint.
    r_instance_ids: Instance IDs for right geometries.
    strength_ratios: Strength ratio for each joint.

Returns:
    SimplicialComplex: The joint geometry mesh.)");

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
           py::array_t<IndexT>          l_instance_ids,
           py::list                     r_geo_slots,
           py::array_t<IndexT>          r_instance_ids,
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

            self.apply_to(edges,
                          span{l_slots},
                          as_span<IndexT>(l_instance_ids),
                          span{r_slots},
                          as_span<IndexT>(r_instance_ids),
                          as_span<Float>(strength_ratios));
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("strength_ratios"),
        py::doc(R"(Create joint between two affine bodies (multi-instance mode).
sc: Every edge in the simplicial complex is treated as a joint axis.
l_geo_slots: List of left geometry slots for each joint.
l_instance_ids: List of instance IDs for left geometries.
r_geo_slots: List of right geometry slots for each joint.
r_instance_ids: List of instance IDs for right geometries.
strength_ratios: List of strength ratios for each joint (one per edge).)"));
}
}  // namespace pyuipc::constitution
