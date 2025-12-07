#include <pyuipc/geometry/implicit_geometry.h>
#include <uipc/geometry/implicit_geometry.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyImplicitGeometry::PyImplicitGeometry(py::module& m)
{
    auto class_ImplicitGeometry =
        py::class_<ImplicitGeometry, Geometry, S<ImplicitGeometry>>(m, "ImplicitGeometry",
                                                                     R"(ImplicitGeometry class representing implicit surfaces (defined by functions, e.g., planes, spheres).)");

    class_ImplicitGeometry.def(py::init<>(),
                               R"(Create an empty implicit geometry.)");

    class_ImplicitGeometry.def("__repr__",
                               [](const ImplicitGeometry& self)
                               { return fmt::format("{}", self); },
                               R"(String representation of the implicit geometry.
Returns:
    str: String representation.)");
}
}  // namespace pyuipc::geometry
