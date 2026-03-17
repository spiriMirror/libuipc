#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyEngine
{
  public:
    PyEngine(py::module_& m);
};

class PyPyIEngine
{
  public:
    PyPyIEngine(py::module_& m);
};
}  // namespace pyuipc::core
