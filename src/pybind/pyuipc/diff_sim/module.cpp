#include <pyuipc/diff_sim/module.h>
#include <pyuipc/diff_sim/sparse_coo_view.h>
#include <pyuipc/diff_sim/parameter_collection.h>
namespace pyuipc::diff_sim
{
PyModule::PyModule(py::module& m)
{
    PySparseCOOView{m};
    PyParameterCollection{m};
}
}  // namespace pyuipc::diff_sim
