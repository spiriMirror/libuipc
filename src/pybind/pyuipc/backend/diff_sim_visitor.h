#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyDiffSimVisitor
{
  public:
    PyDiffSimVisitor(py::module& m);
};
}  // namespace pyuipc::backend
