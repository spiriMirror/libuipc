#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyAnimatorVisitor
{
  public:
    PyAnimatorVisitor(py::module_& m);
};
}  // namespace pyuipc::backend

