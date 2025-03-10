#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyFeature
{
  public:
    PyFeature(py::module& m);
};
}  // namespace pyuipc::core
