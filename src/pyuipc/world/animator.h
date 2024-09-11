#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::world
{
class PyAnimator
{
  public:
    PyAnimator(py::module& m);
};
}  // namespace pyuipc::world