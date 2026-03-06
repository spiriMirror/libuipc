#pragma once
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <functional>
#include <uipc/common/type_define.h>
#include <pyuipc/exception.h>
#include <pyuipc/as_numpy.h>
#include <pyuipc/common/json.h>

namespace pyuipc
{
using namespace uipc;
namespace py = nanobind;

extern py::module_& top_module();
}  // namespace pyuipc
