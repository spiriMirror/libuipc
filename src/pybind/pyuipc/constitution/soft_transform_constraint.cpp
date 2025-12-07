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
        py::class_<SoftTransformConstraint, Constraint>(m, "SoftTransformConstraint",
                                                        R"(SoftTransformConstraint for constraining affine transforms with soft (penalty-based) constraints.)");

    class_SoftTransformConstraint
        .def(py::init<const Json&>(),
             py::arg("config") = SoftTransformConstraint::default_config(),
             R"(Create a SoftTransformConstraint.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)")
        .def(
            "apply_to",
            [](SoftTransformConstraint& self, geometry::SimplicialComplex& sc, py::array_t<Float> strength_rate)
            { self.apply_to(sc, to_matrix<Vector2>(strength_rate)); },
            py::arg("sc"),
            py::arg("strength_rate") = as_numpy(Vector2{100.0, 100}),
            R"(Apply soft transform constraint to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    strength_rate: 2D vector [translation_strength, rotation_strength] (default: [100.0, 100]).)")
        .def_static("default_config", &SoftTransformConstraint::default_config,
                   R"(Get the default SoftTransformConstraint configuration.
Returns:
    dict: Default configuration dictionary.)");

    auto class_RotatingMotor = py::class_<RotatingMotor, Constraint>(m, "RotatingMotor",
                                                                     R"(RotatingMotor constraint for applying rotational motion to affine bodies.)");

    class_RotatingMotor
        .def(py::init<const Json&>(), py::arg("config") = RotatingMotor::default_config(),
             R"(Create a RotatingMotor.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)")
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
            py::arg("motor_rot_vel") = 2 * std::numbers::pi,
            R"(Apply rotating motor to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    strength: Motor strength (default: 100.0).
    motor_axis: Rotation axis vector (default: X-axis).
    motor_rot_vel: Rotational velocity in rad/s (default: 2π).)")
        .def_static("default_config", &RotatingMotor::default_config,
                   R"(Get the default RotatingMotor configuration.
Returns:
    dict: Default configuration dictionary.)")
        .def_static("animate", &RotatingMotor::animate, py::arg("sc"), py::arg("dt"),
                   R"(Animate the rotating motor for a time step.
Args:
    sc: SimplicialComplex to animate.
    dt: Time step.)");

    auto class_LinearMotor = py::class_<LinearMotor, Constraint>(m, "LinearMotor",
                                                                  R"(LinearMotor constraint for applying linear motion to affine bodies.)");

    class_LinearMotor
        .def(py::init<const Json&>(), py::arg("config") = LinearMotor::default_config(),
             R"(Create a LinearMotor.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)")
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
            py::arg("motor_vel")  = 1.0,
            R"(Apply linear motor to a simplicial complex.
Args:
    sc: SimplicialComplex to apply to.
    strength: Motor strength (default: 100.0).
    motor_axis: Motion direction vector (default: -Z-axis).
    motor_vel: Linear velocity (default: 1.0).)")
        .def_static("default_config", &LinearMotor::default_config,
                   R"(Get the default LinearMotor configuration.
Returns:
    dict: Default configuration dictionary.)")
        .def_static("animate", &LinearMotor::animate, py::arg("sc"), py::arg("dt"),
                   R"(Animate the linear motor for a time step.
Args:
    sc: SimplicialComplex to animate.
    dt: Time step.)");
}
}  // namespace pyuipc::constitution
