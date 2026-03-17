#include <pyuipc/builtin/constants.h>
#include <uipc/builtin/constants.h>

namespace pyuipc::builtin
{
PyConstants::PyConstants(py::module_& m)
{
    m.attr("adaptive") = uipc::builtin::adaptive;
}
}  // namespace pyuipc::builtin