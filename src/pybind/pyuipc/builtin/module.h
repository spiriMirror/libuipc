#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::builtin
{
class PyModule
{
  public:
    PyModule(py::module& m);
};
}  // namespace pyuipc::builtin
