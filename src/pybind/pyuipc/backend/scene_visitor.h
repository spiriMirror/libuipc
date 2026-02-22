#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PySceneVisitor
{
  public:
    PySceneVisitor(py::module_& m);
};
}  // namespace pyuipc::backend