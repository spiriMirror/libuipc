#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PyAnimator
{
  public:
    PyAnimator(py::module& m);
};
}  // namespace pyuipc::core