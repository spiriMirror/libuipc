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

    class_ContactSystemFeature.def(
        "contact_energy",
        [](ContactSystemFeature& self, std::string_view prim_type, geometry::Geometry& energy)
        { self.contact_energy(prim_type, energy); },
        py::arg("prim_type"),
        py::arg("energy"));

    class_ContactSystemFeature.def(
        "contact_gradient",
        [](ContactSystemFeature& self, std::string_view prim_type, geometry::Geometry& vert_grad)
        { self.contact_gradient(prim_type, vert_grad); },
        py::arg("prim_type"),
        py::arg("vert_grad"));

    class_ContactSystemFeature.def(
        "contact_hessian",
        [](ContactSystemFeature& self, std::string_view prim_type, geometry::Geometry& vert_hess)
        { self.contact_hessian(prim_type, vert_hess); },
        py::arg("prim_type"),
        py::arg("vert_hess"));

    class_ContactSystemFeature.def(
        "contact_energy",
        [](ContactSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& prims)
        { self.contact_energy(c, prims); },
        py::arg("constitution"),
        py::arg("prims"));

    class_ContactSystemFeature.def(
        "contact_gradient",
        [](ContactSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_grad)
        { self.contact_gradient(c, vert_grad); },
        py::arg("constitution"),
        py::arg("vert_grad"));

    class_ContactSystemFeature.def(
        "contact_hessian",
        [](ContactSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_hess)
        { self.contact_hessian(c, vert_hess); },
        py::arg("constitution"),
        py::arg("vert_hess"));

    class_ContactSystemFeature.def("contact_primitive_types",
                                   &ContactSystemFeature::contact_primitive_types);

    class_ContactSystemFeature.attr("FeatureName") = ContactSystemFeature::FeatureName;
}
}  // namespace pyuipc::core