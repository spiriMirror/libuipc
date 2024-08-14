#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::engine
{
class PyEngine
{
  public:
    PyEngine(py::module& m);
};
}  // namespace pyuipc::engine
