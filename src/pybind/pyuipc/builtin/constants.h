#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::builtin
{
class PyConstants
{
  public:
    PyConstants(py::module_& m);
};
}  // namespace pyuipc::builtin