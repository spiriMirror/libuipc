#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyModule
{
  public:
    PyModule(py::module_& m);
};
}  // namespace pyuipc::backend