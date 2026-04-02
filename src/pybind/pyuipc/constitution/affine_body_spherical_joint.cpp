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
while allowing free relative rotation.)");

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

    // World create_geometry
    class_AffineBodySphericalJoint.def(
        "create_geometry",
        [](AffineBodySphericalJoint&    self,
           py::array_t<Float>           positions,
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
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            for(auto item : r_geo_slots)
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));

            return self.create_geometry(as_span_of<Vector3>(positions),
                                        span{l_slots},
                                        as_span<IndexT>(l_instance_ids),
                                        span{r_slots},
                                        as_span<IndexT>(r_instance_ids),
                                        as_span<Float>(strength_ratios));
        },
        py::arg("positions"),
        py::arg("l_geo_slots"),
        py::arg("l_instance_ids"),
        py::arg("r_geo_slots"),
        py::arg("r_instance_ids"),
        py::arg("strength_ratios"),
        R"(Create spherical joint geometry using world-space anchor positions.

Args:
    positions: World positions of the anchor point for each joint (Nx3 array).
    l_geo_slots: Left geometry slots for each joint.
    l_instance_ids: Instance IDs for left geometries.
    r_geo_slots: Right geometry slots for each joint.
    r_instance_ids: Instance IDs for right geometries.
    strength_ratios: Strength ratio for each joint.

Returns:
    SimplicialComplex: The joint geometry mesh.)");

    // Local create_geometry
    class_AffineBodySphericalJoint.def(
        "create_geometry",
        [](AffineBodySphericalJoint&    self,
           py::array_t<Float>           l_positions,
           py::array_t<Float>           r_positions,
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
        R"(Create spherical joint geometry using local-space anchor positions.

Args:
    l_positions: Local-space anchor positions on the left body (Nx3 array).
    r_positions: Local-space anchor positions on the right body (Nx3 array).
    l_geo_slots: Left geometry slots for each joint.
    l_instance_ids: Instance IDs for left geometries.
    r_geo_slots: Right geometry slots for each joint.
    r_instance_ids: Instance IDs for right geometries.
    strength_ratios: Strength ratio for each joint.

Returns:
    SimplicialComplex: The joint geometry mesh.)");

    // Single-instance apply_to
    class_AffineBodySphericalJoint.def(
        "apply_to",
        [](AffineBodySphericalJoint&    self,
           geometry::SimplicialComplex& sc,
           py::list                     l_geo_slots,
           py::list                     r_geo_slots,
           Float                        strength_ratio)
        {
            vector<S<geometry::SimplicialComplexSlot>> l_slots;
            vector<S<geometry::SimplicialComplexSlot>> r_slots;
            l_slots.reserve(l_geo_slots.size());
            r_slots.reserve(r_geo_slots.size());
            for(auto item : l_geo_slots)
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            for(auto item : r_geo_slots)
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));

            self.apply_to(sc, span{l_slots}, span{r_slots}, strength_ratio);
        },
        py::arg("sc"),
        py::arg("l_geo_slots"),
        py::arg("r_geo_slots"),
        py::arg("strength_ratio") = Float{100.0},
        py::doc(R"(Bind spherical joint geometry to affine bodies (single-instance mode).
sc: A SimplicialComplex with vertices representing joint anchor points.
l_geo_slots: Left geometry slots for each joint.
r_geo_slots: Right geometry slots for each joint.
strength_ratio: Stiffness = strength_ratio * (BodyMassA + BodyMassB) for all joints.)"));

    // Multi-instance apply_to
    class_AffineBodySphericalJoint.def(
        "apply_to",
        [](AffineBodySphericalJoint&    self,
           geometry::SimplicialComplex& sc,
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
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            for(auto item : r_geo_slots)
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));

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
        py::doc(R"(Bind spherical joint geometry to affine bodies (multi-instance mode).
sc: A SimplicialComplex with vertices representing joint anchor points.
l_geo_slots: Left geometry slots for each joint.
l_instance_ids: Instance IDs for left geometries.
r_geo_slots: Right geometry slots for each joint.
r_instance_ids: Instance IDs for right geometries.
strength_ratios: Strength ratios for each joint (one per vertex).)"));
}
}  // namespace pyuipc::constitution
