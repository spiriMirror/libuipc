#include <pyuipc/constitution/affine_body_prismatic_joint.h>
#include <uipc/constitution/affine_body_prismatic_joint.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyPrismaticJoint::PyAffineBodyPrismaticJoint(py::module_& m)
{
    auto class_AffineBodyPrismaticJoint =
        py::class_<AffineBodyPrismaticJoint, InterAffineBodyConstitution>(
            m, "AffineBodyPrismaticJoint", R"(AffineBodyPrismaticJoint constitution for prismatic (sliding) joints between affine bodies.)");

    class_AffineBodyPrismaticJoint.def(py::init<const Json&>(),
                                       py::arg("config") =
                                           AffineBodyPrismaticJoint::default_config(),
                                       R"(Create an AffineBodyPrismaticJoint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyPrismaticJoint.def_static("default_config",
                                              &AffineBodyPrismaticJoint::default_config,
                                              R"(Get the default AffineBodyPrismaticJoint configuration.
Returns:
    dict: Default configuration dictionary.)");

    // Deprecated API
    class_AffineBodyPrismaticJoint.def(
        "apply_to",
        [](AffineBodyPrismaticJoint& self, geometry::SimplicialComplex& edges, py::list geo_slot_tuples, Float strength_ratio)
        {
            vector<AffineBodyPrismaticJoint::SlotTuple> geo_slots;
            geo_slots.reserve(geo_slot_tuples.size());
            for(auto item : geo_slot_tuples)
            {
                if(!py::isinstance<py::tuple>(item))
                {
                    throw std::invalid_argument("geo_slot_tuples must be a list of tuples");
                }
                geo_slots.push_back(py::cast<AffineBodyPrismaticJoint::SlotTuple>(item));
            }
            self.apply_to(edges, geo_slots, strength_ratio);
        },
        py::arg("sc"),
        py::arg("geo_slot_tuples"),
        py::arg("strength_ratio") = Float{100.0},
        py::doc(R"(Deprecated: Create joint between two affine bodies (old API).
sc: Every edge in the simplicial complex is treated as a joint axis.
geo_slot_tuples: A list of tuples, each containing two SimplicialComplexSlot objects, telling who are linked by the joint.
strength_ratio: Stiffness = strength_ratio * (BodyMassA + BodyMassB))"));

    // New API - Single instance mode
    class_AffineBodyPrismaticJoint.def(
        "apply_to",
        [](AffineBodyPrismaticJoint&    self,
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

    // New API - Multi-instance mode
    class_AffineBodyPrismaticJoint.def(
        "apply_to",
        [](AffineBodyPrismaticJoint&    self,
           geometry::SimplicialComplex& edges,
           py::list                     l_geo_slots,
           py::list                     l_instance_id,
           py::list                     r_geo_slots,
           py::list                     r_instance_id,
           py::list                     strength_ratio)
        {
            vector<S<geometry::SimplicialComplexSlot>> l_slots;
            vector<IndexT>                             l_instances;
            vector<S<geometry::SimplicialComplexSlot>> r_slots;
            vector<IndexT>                             r_instances;
            vector<Float>                              strength_ratios;

            l_slots.reserve(l_geo_slots.size());
            l_instances.reserve(l_instance_id.size());
            r_slots.reserve(r_geo_slots.size());
            r_instances.reserve(r_instance_id.size());
            strength_ratios.reserve(strength_ratio.size());

            for(auto item : l_geo_slots)
            {
                l_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            }
            for(auto item : l_instance_id)
            {
                l_instances.push_back(py::cast<IndexT>(item));
            }
            for(auto item : r_geo_slots)
            {
                r_slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));
            }
            for(auto item : r_instance_id)
            {
                r_instances.push_back(py::cast<IndexT>(item));
            }
            for(auto item : strength_ratio)
            {
                strength_ratios.push_back(py::cast<Float>(item));
            }
            self.apply_to(edges,
                          span{l_slots},
                          span{l_instances},
                          span{r_slots},
                          span{r_instances},
                          span{strength_ratios});
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
