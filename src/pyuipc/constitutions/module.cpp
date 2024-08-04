#include <pyuipc/constitutions/module.h>
#include <pyuipc/constitutions/affine_body.h>
namespace pyuipc::constitution
{
Module::Module(py::module& m)
{
    PyAffineBodyConstitution{m};
}
}  // namespace pyuipc::constitution
