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
           py::list                      lowers,
           py::list                      uppers,
           py::list                      strengths)
        {
            vector<Float> lower_v;
            vector<Float> upper_v;
            vector<Float> strength_v;

            lower_v.reserve(lowers.size());
            upper_v.reserve(uppers.size());
            strength_v.reserve(strengths.size());

            for(auto v : lowers)
                lower_v.push_back(py::cast<Float>(v));
            for(auto v : uppers)
                upper_v.push_back(py::cast<Float>(v));
            for(auto v : strengths)
                strength_v.push_back(py::cast<Float>(v));

            self.apply_to(sc, span{lower_v}, span{upper_v}, span{strength_v});
        },
        py::arg("sc"),
        py::arg("lowers"),
        py::arg("uppers"),
        py::arg("strengths"),
        R"(Apply per-edge lower/upper/strength vectors.)");
}
}  // namespace pyuipc::constitution
