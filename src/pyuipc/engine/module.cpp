#include <pyuipc/engine/module.h>
#include <pyuipc/engine/engine.h>
namespace pyuipc::engine
{
Module::Module(py::module& m)
{
    PyEngine{m};
}
}  // namespace pyuipc::engine
