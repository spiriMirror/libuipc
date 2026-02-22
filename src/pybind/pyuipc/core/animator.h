#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyAnimator
{
  public:
    PyAnimator(py::module_& m);
};
}  // namespace pyuipc::core