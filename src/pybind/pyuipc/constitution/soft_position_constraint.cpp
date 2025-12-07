#include <pyuipc/constitution/soft_position_constraint.h>
#include <uipc/constitution/soft_position_constraint.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PySoftPositionConstraint::PySoftPositionConstraint(py::module& m)
{
    auto class_SoftPositionConstraint =
        py::class_<SoftPositionConstraint, Constraint>(m, "SoftPositionConstraint",
                                                        R"(SoftPositionConstraint for constraining vertex positions with soft (penalty-based) constraints.)");

    class_SoftPositionConstraint
        .def(py::init<const Json&>(),
             py::arg("config") = SoftPositionConstraint::default_config(),
             R"(Create a SoftPositionConstraint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)")
        .def("apply_to", &SoftPositionConstraint::apply_to, py::arg("sc"), py::arg("strength_rate") = 100.0,
             R"(Apply soft position constraint to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    strength_rate: Constraint strength rate (default: 100.0).)")
        .def_static("default_config", &SoftPositionConstraint::default_config,
                   R"(Get the default SoftPositionConstraint configuration.
Returns:
    dict: Default configuration dictionary.)");
}
}  // namespace pyuipc::constitution
