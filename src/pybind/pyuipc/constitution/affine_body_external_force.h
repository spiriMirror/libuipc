#pragma once
#include <pyuipc/pyuipc.h>

namespace pyuipc::constitution
{
void bind_affine_body_external_force(py::module& m);

struct PyAffineBodyExternalForce
{
    PyAffineBodyExternalForce(py::module& m) { bind_affine_body_external_force(m); }
};
}  // namespace pyuipc::constitution
