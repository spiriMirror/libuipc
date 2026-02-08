#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::builtin
{
class PyConstants
{
  public:
    PyConstants(py::module& m);
};
}  // namespace pyuipc::builtin