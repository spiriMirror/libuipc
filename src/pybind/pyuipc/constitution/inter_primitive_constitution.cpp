#include <pyuipc/constitution/inter_primitive_constitution.h>
#include <uipc/constitution/inter_primitive_constitution.h>

namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyInterPrimitiveConstitution::PyInterPrimitiveConstitution(py::module& m)
{
    auto class_InterPrimitiveConstitution =
        py::class_<InterPrimitiveConstitution, IConstitution>(m, "InterPrimitiveConstitution");
}
}  // namespace pyuipc::constitution