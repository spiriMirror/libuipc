#include <pyuipc/constitution/soft_transform_constraint.h>
#include <uipc/constitution/soft_transform_constraint.h>
#include <pyuipc/common/json.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PySoftTransformConstraint::PySoftTransformConstraint(py::module& m)
{
    auto class_SoftTransformConstraint =
        py::class_<SoftTransformConstraint, Constraint>(m, "SoftTransformConstraint");

    class_SoftTransformConstraint
        .def(py::init<const Json&>(),
             py::arg("config") = SoftTransformConstraint::default_config())
        .def("apply_to",
             &SoftTransformConstraint::apply_to,
             py::arg("sc"),
             py::arg("strength_rate") = as_numpy(Vector2{100.0, 100}))
        .def_static("default_config", &SoftTransformConstraint::default_config);
}
}  // namespace pyuipc::constitution
