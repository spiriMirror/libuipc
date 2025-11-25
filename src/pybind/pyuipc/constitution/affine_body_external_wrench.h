#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::constitution
{
void bind_affine_body_external_wrench(py::module& m);

struct PyAffineBodyExternalWrench
{
    PyAffineBodyExternalWrench(py::module& m) { bind_affine_body_external_wrench(m); }
};
}  // namespace pyuipc::constitution
