#include <pyuipc/core/contact_system_feature.h>
#include <uipc/core/contact_system_feature.h>
#include <pybind11/stl.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyContactSystemFeature::PyContactSystemFeature(py::module& m)
{
    auto class_ContactSystemFeature =
        py::class_<ContactSystemFeature, IFeature, S<ContactSystemFeature>>(m, "ContactSystemFeature",
                                                                             R"(Feature for computing contact energy, gradients, and Hessians.)");

    class_ContactSystemFeature.def(
        "contact_energy",
        [](ContactSystemFeature& self, std::string_view prim_type, geometry::Geometry& energy)
        { self.contact_energy(prim_type, energy); },
        py::arg("prim_type"),
        py::arg("energy"),
        R"(Compute contact energy for a primitive type.
Args:
    prim_type: Primitive type string.
    energy: Geometry to store energy values (modified in-place).)");

    class_ContactSystemFeature.def(
        "contact_gradient",
        [](ContactSystemFeature& self, std::string_view prim_type, geometry::Geometry& vert_grad)
        { self.contact_gradient(prim_type, vert_grad); },
        py::arg("prim_type"),
        py::arg("vert_grad"),
        R"(Compute contact gradient for a primitive type.
Args:
    prim_type: Primitive type string.
    vert_grad: Geometry to store vertex gradients (modified in-place).)");

    class_ContactSystemFeature.def(
        "contact_hessian",
        [](ContactSystemFeature& self, std::string_view prim_type, geometry::Geometry& vert_hess)
        { self.contact_hessian(prim_type, vert_hess); },
        py::arg("prim_type"),
        py::arg("vert_hess"),
        R"(Compute contact Hessian for a primitive type.
Args:
    prim_type: Primitive type string.
    vert_hess: Geometry to store vertex Hessians (modified in-place).)");

    class_ContactSystemFeature.def(
        "contact_energy",
        [](ContactSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& prims)
        { self.contact_energy(c, prims); },
        py::arg("constitution"),
        py::arg("prims"),
        R"(Compute contact energy for a constitution.
Args:
    constitution: Constitution to compute energy for.
    prims: Geometry containing primitives (modified in-place with energy values).)");

    class_ContactSystemFeature.def(
        "contact_gradient",
        [](ContactSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_grad)
        { self.contact_gradient(c, vert_grad); },
        py::arg("constitution"),
        py::arg("vert_grad"),
        R"(Compute contact gradient for a constitution.
Args:
    constitution: Constitution to compute gradient for.
    vert_grad: Geometry to store vertex gradients (modified in-place).)");

    class_ContactSystemFeature.def(
        "contact_hessian",
        [](ContactSystemFeature& self, const constitution::IConstitution& c, geometry::Geometry& vert_hess)
        { self.contact_hessian(c, vert_hess); },
        py::arg("constitution"),
        py::arg("vert_hess"),
        R"(Compute contact Hessian for a constitution.
Args:
    constitution: Constitution to compute Hessian for.
    vert_hess: Geometry to store vertex Hessians (modified in-place).)");

    class_ContactSystemFeature.def("contact_primitive_types",
                                   &ContactSystemFeature::contact_primitive_types,
                                   R"(Get the list of supported primitive types.
Returns:
    list: List of primitive type strings.)");

    class_ContactSystemFeature.attr("FeatureName") = ContactSystemFeature::FeatureName;
}
}  // namespace pyuipc::core