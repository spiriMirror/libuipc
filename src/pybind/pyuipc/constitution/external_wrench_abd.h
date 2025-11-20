#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::constitution
{
void bind_external_wrench_abd(py::module& m);

struct PyExternalWrenchABD
{
    PyExternalWrenchABD(py::module& m) { bind_external_wrench_abd(m); }
};
}  // namespace pyuipc::constitution
