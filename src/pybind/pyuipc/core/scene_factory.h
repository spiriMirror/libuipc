#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PySceneFactory
{
  public:
    PySceneFactory(py::module_& m);
};
}  // namespace pyuipc::core
