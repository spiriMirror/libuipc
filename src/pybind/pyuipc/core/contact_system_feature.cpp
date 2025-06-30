#include <pyuipc/core/contact_system_feature.h>
#include <uipc/core/contact_system_feature.h>
#include <pybind11/stl.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyContactSystemFeature::PyContactSystemFeature(py::module& m)
{
    auto class_ContactSystemFeature =
        py::class_<ContactSystemFeature, IFeature, S<ContactSystemFeature>>(m, "ContactSystemFeature");

    class_ContactSystemFeature.def("contact_gradient",
                                   &ContactSystemFeature::contact_gradient,
                                   py::arg("vert_grad"));

    class_ContactSystemFeature.def("contact_hessian",
                                   &ContactSystemFeature::contact_hessian,
                                   py::arg("vert_hess"));

    class_ContactSystemFeature.def("contact_primitives",
                                   &ContactSystemFeature::contact_primitives,
                                   py::arg("prim_type"),
                                   py::arg("prims"));

    class_ContactSystemFeature.def("contact_primitive_types",
                                   &ContactSystemFeature::contact_primitive_types);

    class_ContactSystemFeature.attr("FeatureName") = ContactSystemFeature::FeatureName;
}
}  // namespace pyuipc::core