#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyDiffSimVisitor
{
  public:
    PyDiffSimVisitor(py::module_& m);
};
}  // namespace pyuipc::backend
