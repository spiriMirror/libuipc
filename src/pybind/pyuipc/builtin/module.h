#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::builtin
{
class PyModule
{
  public:
    PyModule(py::module_& m);
};
}  // namespace pyuipc::builtin
