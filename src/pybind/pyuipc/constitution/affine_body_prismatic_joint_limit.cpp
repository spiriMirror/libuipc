#include <pyuipc/constitution/affine_body_prismatic_joint_limit.h>
#include <uipc/constitution/affine_body_prismatic_joint_limit.h>
#include <uipc/constitution/inter_affine_body_extra_constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;

PyAffineBodyPrismaticJointLimit::PyAffineBodyPrismaticJointLimit(py::module& m)
{
    auto class_AffineBodyPrismaticJointLimit =
        py::class_<AffineBodyPrismaticJointLimit, InterAffineBodyExtraConstitution>(
            m,
            "AffineBodyPrismaticJointLimit",
            R"(AffineBodyPrismaticJointLimit cubic penalty limit on prismatic joint value.)");

    class_AffineBodyPrismaticJointLimit.def(
        py::init<const Json&>(),
        py::arg("config") = AffineBodyPrismaticJointLimit::default_config(),
        R"(Create an AffineBodyPrismaticJointLimit.
Args:
    config: Configuration dictionary (optional).)");

    class_AffineBodyPrismaticJointLimit.def_static(
        "default_config",
        &AffineBodyPrismaticJointLimit::default_config,
        R"(Get default configuration.)");

    class_AffineBodyPrismaticJointLimit.def(
        "apply_to",
        py::overload_cast<geometry::SimplicialComplex&, Float, Float, Float>(
            &AffineBodyPrismaticJointLimit::apply_to),
        py::arg("sc"),
        py::arg("lower"),
        py::arg("upper"),
        py::arg("strength") = Float{1},
        R"(Apply scalar lower/upper/strength to all joint edges.)");

    class_AffineBodyPrismaticJointLimit.def(
        "apply_to",
        [](AffineBodyPrismaticJointLimit& self,
           geometry::SimplicialComplex&   sc,
           py::array_t<Float>             lowers,
           py::array_t<Float>             uppers,
           py::array_t<Float>             strengths)
        { self.apply_to(sc, as_span<Float>(lowers), as_span<Float>(uppers), as_span<Float>(strengths)); },
        py::arg("sc"),
        py::arg("lowers"),
        py::arg("uppers"),
        py::arg("strengths"),
        R"(Apply per-edge lower/upper/strength vectors.)");
}
}  // namespace pyuipc::constitution
