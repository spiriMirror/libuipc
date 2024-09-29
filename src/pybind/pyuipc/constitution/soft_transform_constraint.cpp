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
        .def(
            "apply_to",
            [](SoftTransformConstraint& self, geometry::SimplicialComplex& sc, py::array_t<Float> strength_rate)
            { self.apply_to(sc, to_matrix<Vector2>(strength_rate)); },
            py::arg("sc"),
            py::arg("strength_rate") = as_numpy(Vector2{100.0, 100}))
        .def_static("default_config", &SoftTransformConstraint::default_config);

    auto class_RotatingMotor = py::class_<RotatingMotor, Constraint>(m, "RotatingMotor");

    class_RotatingMotor
        .def(py::init<const Json&>(), py::arg("config") = RotatingMotor::default_config())
        .def(
            "apply_to",
            [](RotatingMotor&               self,
               geometry::SimplicialComplex& sc,
               Float                        strength,
               py::array_t<Float>           motor_axis,
               Float                        motor_rot_vel) {
                self.apply_to(sc, strength, to_matrix<Vector3>(motor_axis), motor_rot_vel);
            },
            py::arg("sc"),
            py::arg("strength")      = 100.0,
            py::arg("motor_axis")    = as_numpy(Vector3::UnitX().eval()),
            py::arg("motor_rot_vel") = 2 * std::numbers::pi)
        .def_static("default_config", &RotatingMotor::default_config)
        .def_static("animate", &RotatingMotor::animate, py::arg("sc"), py::arg("dt"));

    auto class_LinearMotor = py::class_<LinearMotor, Constraint>(m, "LinearMotor");

    class_LinearMotor
        .def(py::init<const Json&>(), py::arg("config") = LinearMotor::default_config())
        .def(
            "apply_to",
            [](LinearMotor&                 self,
               geometry::SimplicialComplex& sc,
               Float                        strength,
               py::array_t<Float>           motor_axis,
               Float                        motor_vel) {
                self.apply_to(sc, strength, to_matrix<Vector3>(motor_axis), motor_vel);
            },
            py::arg("sc"),
            py::arg("strength")   = 100.0,
            py::arg("motor_axis") = as_numpy(Vector3{-Vector3::UnitZ()}),
            py::arg("motor_vel")  = 1.0)
        .def_static("default_config", &LinearMotor::default_config)
        .def_static("animate", &LinearMotor::animate, py::arg("sc"), py::arg("dt"));
}
}  // namespace pyuipc::constitution
