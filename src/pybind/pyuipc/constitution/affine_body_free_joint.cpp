#include <pyuipc/constitution/affine_body_free_joint.h>
#include <uipc/constitution/affine_body_free_joint.h>
#include <uipc/constitution/constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyFreeJoint::PyAffineBodyFreeJoint(py::module& m)
{
    auto class_AffineBodyFreeJoint =
        py::class_<AffineBodyFreeJoint, InterAffineBodyConstitution>(
            m, "AffineBodyFreeJoint", R"(AffineBodyFreeJoint constitution for 6-DOF free joints connecting an affine body to the implicit world origin.)");

    class_AffineBodyFreeJoint.def(py::init<const Json&>(),
                                  py::arg("config") = AffineBodyFreeJoint::default_config(),
                                  R"(Create an AffineBodyFreeJoint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    class_AffineBodyFreeJoint.def_static("default_config",
                                         &AffineBodyFreeJoint::default_config,
                                         R"(Get the default AffineBodyFreeJoint configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_AffineBodyFreeJoint.def(
        "create_geometry",
        [](AffineBodyFreeJoint& self, py::list geo_slots, py::array_t<IndexT> instance_ids)
        {
            vector<S<geometry::SimplicialComplexSlot>> slots;
            slots.reserve(geo_slots.size());
            for(auto item : geo_slots)
                slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));

            return self.create_geometry(span{slots}, as_span<IndexT>(instance_ids));
        },
        py::arg("geo_slots"),
        py::arg("instance_ids"),
        R"(Create free joint geometry for the given bodies.

Each body gets 6 vertices (one per DOF: 3 translational + 3 rotational).

Args:
    geo_slots: Geometry slots for each body to attach a free joint to.
    instance_ids: Instance IDs for each geometry.

Returns:
    SimplicialComplex: The free joint geometry mesh.)");

    class_AffineBodyFreeJoint.def(
        "apply_to",
        [](AffineBodyFreeJoint&         self,
           geometry::SimplicialComplex& sc,
           py::list                     geo_slots,
           py::array_t<IndexT>          instance_ids)
        {
            vector<S<geometry::SimplicialComplexSlot>> slots;
            slots.reserve(geo_slots.size());
            for(auto item : geo_slots)
                slots.push_back(py::cast<S<geometry::SimplicialComplexSlot>>(item));

            self.apply_to(sc, span{slots}, as_span<IndexT>(instance_ids));
        },
        py::arg("sc"),
        py::arg("geo_slots"),
        py::arg("instance_ids"),
        R"(Write free joint attributes to existing vertices.

Args:
    sc: A SimplicialComplex with 6*N pre-allocated vertices.
    geo_slots: Geometry slots for each body.
    instance_ids: Instance IDs for each geometry.)");
}
}  // namespace pyuipc::constitution
