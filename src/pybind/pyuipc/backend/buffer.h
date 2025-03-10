#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::backend
{
class PyBuffer
{
  public:
    PyBuffer(py::module& m);
};
}  // namespace pyuipc::backend