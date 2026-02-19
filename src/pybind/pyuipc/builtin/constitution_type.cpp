#include <pyuipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_type.h>
#include <string>

namespace pyuipc::builtin
{
PyConstitutionType::PyConstitutionType(py::module& m)
{
    m.attr("AffineBody") = std::string(uipc::builtin::AffineBody);
    m.attr("FiniteElement") = std::string(uipc::builtin::FiniteElement);
    m.attr("Constraint") = std::string(uipc::builtin::Constraint);
    m.attr("InterAffineBody") = std::string(uipc::builtin::InterAffineBody);
    m.attr("InterPrimitive") = std::string(uipc::builtin::InterPrimitive);
}
}  // namespace pyuipc::builtin

