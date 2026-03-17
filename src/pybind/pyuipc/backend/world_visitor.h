#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyWorldVisitor
{
  public:
    PyWorldVisitor(py::module_& m);
};
}  // namespace pyuipc::backend