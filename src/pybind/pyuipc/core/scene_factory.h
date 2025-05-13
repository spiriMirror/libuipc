#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PySceneFactory
{
  public:
    PySceneFactory(py::module& m);
};
}  // namespace pyuipc::core
