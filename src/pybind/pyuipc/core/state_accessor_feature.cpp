#include <pyuipc/core/state_accessor_feature.h>
#include <uipc/core/finite_element_state_accessor_feature.h>
#include <uipc/core/affine_body_state_accessor_feature.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyStateAccessorFeature::PyStateAccessorFeature(py::module& m)
{
    auto class_FiniteElementStateAccessorFeature =
        py::class_<FiniteElementStateAccessorFeature, IFeature, S<FiniteElementStateAccessorFeature>>(
            m, "FiniteElementStateAccessorFeature");

    class_FiniteElementStateAccessorFeature.def(
        "vertex_count", &FiniteElementStateAccessorFeature::vertex_count);

    class_FiniteElementStateAccessorFeature.def("create_geometry",
                                                &FiniteElementStateAccessorFeature::create_geometry,
                                                py::arg("vertex_offset") = 0,
                                                py::arg("vertex_count") = ~0ull);

    class_FiniteElementStateAccessorFeature.def(
        "copy_from", &FiniteElementStateAccessorFeature::copy_from, py::arg("state_geo"));

    class_FiniteElementStateAccessorFeature.def(
        "copy_to", &FiniteElementStateAccessorFeature::copy_to, py::arg("state_geo"));

    class_FiniteElementStateAccessorFeature.attr("FeatureName") =
        FiniteElementStateAccessorFeature::FeatureName;


    auto class_AffineBodyStateAccessorFeature =
        py::class_<AffineBodyStateAccessorFeature, IFeature, S<AffineBodyStateAccessorFeature>>(
            m, "AffineBodyStateAccessorFeature");

    class_AffineBodyStateAccessorFeature.def("body_count",
                                             &AffineBodyStateAccessorFeature::body_count);

    class_AffineBodyStateAccessorFeature.def("create_geometry",
                                             &AffineBodyStateAccessorFeature::create_geometry,
                                             py::arg("body_offset") = 0,
                                             py::arg("body_count")  = ~0ull);

    class_AffineBodyStateAccessorFeature.def("copy_from",
                                             &AffineBodyStateAccessorFeature::copy_from,
                                             py::arg("state_geo"));

    class_AffineBodyStateAccessorFeature.def(
        "copy_to", &AffineBodyStateAccessorFeature::copy_to, py::arg("state_geo"));

    class_AffineBodyStateAccessorFeature.attr("FeatureName") =
        AffineBodyStateAccessorFeature::FeatureName;
}
}  // namespace pyuipc::core
