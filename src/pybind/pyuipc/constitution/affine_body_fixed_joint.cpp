#include <pyuipc/constitution/affine_body_fixed_joint.h>
#include <uipc/constitution/affine_body_fixed_joint.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyFixedJoint::PyAffineBodyFixedJoint(py::module& m)
{
    auto class_AffineBodyFixedJoint =
        py::class_<AffineBodyFixedJoint, InterAffineBodyConstitution>(
            m, "AffineBodyFixedJoint", R"(AffineBodyFixedJoint constitution for fixed (rigid) joints between affine bodies.)");

    class_AffineBodyFixedJoint.def(py::init<const Json&>(),
                                   py::arg("config") =
                                       AffineBodyFixedJoint::default_config(),
                                   R"(Create an AffineBodyFixedJoint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyFixedJoint.def_static("default_config",
                                          &AffineBodyFixedJoint::default_config,
                                          R"(Get the default AffineBodyFixedJoint configuration.
Returns:
    dict: Default configuration dictionary.)");

    // World create_geometry
    class_AffineBodyFixedJoint.def(
        "create_geometry",
        [](AffineBodyFixedJoint&    self,
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

            return self.create_geometry(span{l_slots},
                                        as_span<IndexT>(l_instance_ids),
                                        span{r_slots},
                                        as_span<IndexT>(r_instance_ids),
                                        as_span<Float>(strength_ratios));
        },
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("strength_ratios"),
        R"(Create fixed joint geometry using world-space positions (computed from body transforms).

Args:
    l_geo_slots: Left geometry slots for each joint.
    l_instance_ids: Instance IDs for left geometries.
    r_geo_slots: Right geometry slots for each joint.
    r_instance_ids: Instance IDs for right geometries.
    strength_ratios: Strength ratio for each joint.

Returns:
    SimplicialComplex: The joint geometry mesh.)");

    // Local create_geometry
    class_AffineBodyFixedJoint.def(
        "create_geometry",
        [](AffineBodyFixedJoint&    self,
           py::array_t<Float>       l_positions,
           py::array_t<Float>       r_positions,
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

            return self.create_geometry(as_span_of<Vector3>(l_positions),
                                        as_span_of<Vector3>(r_positions),
                                        span{l_slots},
                                        as_span<IndexT>(l_instance_ids),
                                        span{r_slots},
                                        as_span<IndexT>(r_instance_ids),
                                        as_span<Float>(strength_ratios));
        },
        py::arg("l_positions"),
        py::arg("r_positions"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("strength_ratios"),
        R"(Create fixed joint geometry using local-space attachment positions.

Args:
    l_positions: Local-space attachment positions on the left body (Nx3 array).
    r_positions: Local-space attachment positions on the right body (Nx3 array).
    l_geo_slots: Left geometry slots for each joint.
    l_instance_ids: Instance IDs for left geometries.
    r_geo_slots: Right geometry slots for each joint.
    r_instance_ids: Instance IDs for right geometries.
    strength_ratios: Strength ratio for each joint.

Returns:
    SimplicialComplex: The joint geometry mesh.)");

    //  Single instance mode
    class_AffineBodyFixedJoint.def(
        "apply_to",
        [](AffineBodyFixedJoint&         self,
           geometry::SimplicialComplex&  sc,
           py::list                      l_geo_slots,
           py::list                      r_geo_slots,
           Float                         strength_ratio)
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
            self.apply_to(sc, span{l_slots}, span{r_slots}, strength_ratio);
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("r_geo_slots"),
        py::arg("strength_ratio") = Float{100.0},
        py::doc(R"(Bind fixed joint geometry to affine bodies (single-instance mode).
sc: A SimplicialComplex with vertices representing joint attachment points.
l_geo_slots: Left geometry slots for each joint.
r_geo_slots: Right geometry slots for each joint.
strength_ratio: Stiffness = strength_ratio * (BodyMassA + BodyMassB) for all joints.)"));

    //  Multi-instance mode
    class_AffineBodyFixedJoint.def(
        "apply_to",
        [](AffineBodyFixedJoint&         self,
           geometry::SimplicialComplex&  sc,
           py::list                      l_geo_slots,
           py::array_t<IndexT>           l_instance_ids,
           py::list                      r_geo_slots,
           py::array_t<IndexT>           r_instance_ids,
           py::array_t<Float>            strength_ratios)
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
                          as_span<Float>(strength_ratios));
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("strength_ratios"),
        py::doc(R"(Bind fixed joint geometry to affine bodies (multi-instance mode).
sc: A SimplicialComplex with vertices representing joint attachment points.
l_geo_slots: Left geometry slots for each joint.
l_instance_ids: Instance IDs for left geometries.
r_geo_slots: Right geometry slots for each joint.
r_instance_ids: Instance IDs for right geometries.
strength_ratios: Strength ratios for each joint (one per vertex).)"));
}
}  // namespace pyuipc::constitution