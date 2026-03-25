#include <pyuipc/constitution/affine_body_revolute_joint_limit.h>
#include <uipc/constitution/affine_body_revolute_joint_limit.h>
#include <uipc/constitution/inter_affine_body_extra_constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyRevoluteJointLimit::PyAffineBodyRevoluteJointLimit(py::module& m)
{
    auto class_AffineBodyRevoluteJointLimit =
        py::class_<AffineBodyRevoluteJointLimit, InterAffineBodyExtraConstitution>(
            m,
            "AffineBodyRevoluteJointLimit",
            R"(AffineBodyRevoluteJointLimit cubic penalty limit on revolute joint angle.)");

    class_AffineBodyRevoluteJointLimit.def(
        py::init<const Json&>(),
        py::arg("config") = AffineBodyRevoluteJointLimit::default_config(),
        R"(Create an AffineBodyRevoluteJointLimit.
Args:
    config: Configuration dictionary (optional).)");

    class_AffineBodyRevoluteJointLimit.def_static(
        "default_config",
        &AffineBodyRevoluteJointLimit::default_config,
        R"(Get default configuration.)");

    class_AffineBodyRevoluteJointLimit.def(
        "apply_to",
        py::overload_cast<geometry::SimplicialComplex&, Float, Float, Float>(
            &AffineBodyRevoluteJointLimit::apply_to),
        py::arg("sc"),
        py::arg("lower"),
        py::arg("upper"),
        py::arg("strength") = Float{1},
        R"(Apply scalar lower/upper/strength to all joint edges.)");

    class_AffineBodyRevoluteJointLimit.def(
        "apply_to",
        [](AffineBodyRevoluteJointLimit& self,
           geometry::SimplicialComplex&  sc,
           py::array_t<Float>            lowers,
           py::array_t<Float>            uppers,
           py::array_t<Float>            strengths)
        { self.apply_to(sc, as_span<Float>(lowers), as_span<Float>(uppers), as_span<Float>(strengths)); },
        py::arg("sc"),
        py::arg("lowers"),
        py::arg("uppers"),
        py::arg("strengths"),
        R"(Apply per-edge lower/upper/strength vectors.)");
}
}  // namespace pyuipc::constitution
