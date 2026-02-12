#include <pyuipc/builtin/geometry_type.h>
#include <uipc/builtin/geometry_type.h>
#include <string>

namespace pyuipc::builtin
{
PyGeometryType::PyGeometryType(py::module& m)
{
    m.attr("Geometry") = std::string(uipc::builtin::Geometry);
    m.attr("AbstractSimplicialComplex") = std::string(uipc::builtin::AbstractSimplicialComplex);
    m.attr("SimplicialComplex") = std::string(uipc::builtin::SimplicialComplex);
    m.attr("ImplicitGeometry") = std::string(uipc::builtin::ImplicitGeometry);
}
}  // namespace pyuipc::builtin

