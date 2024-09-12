#include <pyuipc/builtin/attribute_name.h>

#define UIPC_BUILTIN_ATTRIBUTE(name) m.attr(#name) = #name;

namespace pyuipc::builtin
{
PyAttributeName::PyAttributeName(py::module& m)
{
#include <uipc/builtin/details/attribute_name.h>
}
}  // namespace pyuipc::builtin

#undef UIPC_BUILTIN_ATTRIBUTE
