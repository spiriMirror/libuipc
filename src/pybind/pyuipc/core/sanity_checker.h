#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::core
{
class PySanityChecker
{
  public:
    PySanityChecker(py::module_& m);
};

}  // namespace pyuipc::core
