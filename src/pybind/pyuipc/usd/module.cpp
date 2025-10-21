#include <fmt/core.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pyuipc/usd/module.h>

namespace pyuipc::usd
{
PyModule::PyModule(py::module& m)
{
    // According to issue: https://github.com/spiriMirror/libuipc/issues/201
    // Now we don't directly expose USD Module functionality to Python.
}
}  // namespace pyuipc::usd
