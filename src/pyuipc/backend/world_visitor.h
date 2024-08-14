#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyWorldVisitor
{
  public:
    PyWorldVisitor(py::module& m);
};
}  // namespace pyuipc::backend