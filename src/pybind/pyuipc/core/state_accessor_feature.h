#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyStateAccessorFeature
{
  public:
    PyStateAccessorFeature(py::module_& m);
};
}  // namespace pyuipc::core
