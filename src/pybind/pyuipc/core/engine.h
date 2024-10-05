#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyEngine
{
  public:
    PyEngine(py::module& m);
};
}  // namespace pyuipc::core
