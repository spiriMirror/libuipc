#include <pyuipc/constitution/finite_element_constitution.h>
#include <uipc/constitution/finite_element_constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyFiniteElementConstitution::PyFiniteElementConstitution(py::module& m)
{
    auto class_FEMConstitution =
        py::class_<FiniteElementConstitution, IConstitution>(m, "FiniteElementConstitution",
                                                              R"(FiniteElementConstitution base class for finite element material models.)");
}
}  // namespace pyuipc::constitution
