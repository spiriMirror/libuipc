#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PySanityChecker
{
  public:
    PySanityChecker(py::module& m);
};

}  // namespace pyuipc::core
