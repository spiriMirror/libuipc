#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PySceneVisitor
{
  public:
    PySceneVisitor(py::module& m);
};
}  // namespace pyuipc::backend