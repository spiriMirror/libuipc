#include <pyuipc/constitution/fem_constitution.h>
#include <uipc/constitution/fem_constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyFiniteElementConstitution::PyFiniteElementConstitution(py::module& m)
{
    auto class_FEMConstitution =
        py::class_<FiniteElementConstitution, IConstitution>(m, "FEMConstitution");
}
}  // namespace pyuipc::constitution
