#pragma once
#include <pybind11/pybind11.h>
#include <functional>
#include <uipc/common/type_define.h>
#include <pyuipc/exception.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/json.h>

namespace pyuipc
{
using namespace uipc;
namespace py = pybind11;

extern py::module& top_module();
}  // namespace pyuipc
