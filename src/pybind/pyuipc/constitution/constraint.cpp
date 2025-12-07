#include <pyuipc/constitution/constraint.h>
#include <uipc/constitution/constraint.h>
namespace pyuipc::constitution
{
using namespace uipc::constitution;
PyConstraint::PyConstraint(py::module& m)
{

    auto class_Constraint = py::class_<Constraint, IConstitution>(m, "Constraint",
                                                                     R"(Constraint base class for simulation constraints.)");
}
}  // namespace pyuipc::constitution
