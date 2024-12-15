#include <pyuipc/constitution/soft_position_constraint.h>
#include <uipc/constitution/soft_position_constraint.h>
#include <pyuipc/common/json.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PySoftPositionConstraint::PySoftPositionConstraint(py::module& m)
{
    auto class_SoftPositionConstraint =
        py::class_<SoftPositionConstraint, Constraint>(m, "SoftPositionConstraint");

    class_SoftPositionConstraint
        .def(py::init<const Json&>(),
             py::arg("config") = SoftPositionConstraint::default_config())
        .def("apply_to", &SoftPositionConstraint::apply_to, py::arg("sc"), py::arg("strength_rate") = 100.0)
        .def_static("default_config", &SoftPositionConstraint::default_config);
}
}  // namespace pyuipc::constitution
