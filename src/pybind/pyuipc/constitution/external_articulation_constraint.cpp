#include <pyuipc/constitution/external_articulation_constraint.h>
#include <uipc/constitution/external_articulation_constraint.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyExternalArticulationConstraint::PyExternalArticulationConstraint(py::module& m)
{
    auto class_ExternalArticulationConstraint =
        py::class_<ExternalArticulationConstraint, Constraint>(m,
                                                               "ExternalArticulationConstraint",
                                                               R"(External articulation constraint for incorporating external kinetic energy into the IPC system.

Controls motion of affine bodies by prescribing articulation DOFs (joint positions, angles, etc.).
Kinetic energy: K = 1/2(delta_theta - delta_theta_tilde)^T M^t (delta_theta - delta_theta_tilde).

Users must provide M^t (effective mass matrix) and delta_theta_tilde (predicted joint DOF) through
animator callback. Works with AffineBodyRevoluteJoint and AffineBodyPrismaticJoint.)");

    class_ExternalArticulationConstraint
        .def(py::init<const Json&>(),
             py::arg("config") = ExternalArticulationConstraint::default_config(),
             R"(Create an ExternalArticulationConstraint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)")
        .def(
            "create_geometry",
            [](ExternalArticulationConstraint& self, py::list joint_geos)
            {
                vector<S<const geometry::GeometrySlot>> slots;
                slots.reserve(joint_geos.size());
                for(auto item : joint_geos)
                {
                    auto slot = py::cast<S<geometry::GeometrySlot>>(item);
                    slots.push_back(
                        std::static_pointer_cast<const geometry::GeometrySlot>(slot));
                }
                return self.create_geometry(span{slots});
            },
            py::arg("joint_geos"),
            R"(Create geometry for external articulation joints.

Creates geometry with joint attributes (geo_id, index, delta_theta_tilde, delta_theta)
and joint-joint mass attributes. All joints use index 0 by default.

Args:
    joint_geos: List of geometry slots containing joint information.
Returns:
    Geometry: Created geometry with joint constraint data.)")
        .def(
            "create_geometry",
            [](ExternalArticulationConstraint& self, py::list joint_geos, py::list indices)
            {
                vector<S<const geometry::GeometrySlot>> slots;
                vector<IndexT>                          index_vec;
                slots.reserve(joint_geos.size());
                index_vec.reserve(indices.size());

                for(auto item : joint_geos)
                {
                    auto slot = py::cast<S<geometry::GeometrySlot>>(item);
                    slots.push_back(
                        std::static_pointer_cast<const geometry::GeometrySlot>(slot));
                }
                for(auto item : indices)
                {
                    index_vec.push_back(py::cast<IndexT>(item));
                }
                return self.create_geometry(span{slots}, span{index_vec});
            },
            py::arg("joint_geos"),
            py::arg("indices"),
            R"(Create geometry for external articulation joints with specified indices.

Same as single-parameter version but allows selecting specific joints from each slot.

Args:
    joint_geos: List of geometry slots containing joint information.
    indices: List of indices specifying which joint to use from each slot. Must match joint_geos size.
Returns:
    Geometry: Created geometry with joint constraint data.)")
        .def_static("default_config",
                    &ExternalArticulationConstraint::default_config,
                    R"(Get the default ExternalArticulationConstraint configuration.
Returns:
    dict: Default configuration dictionary.)");
}
}  // namespace pyuipc::constitution
