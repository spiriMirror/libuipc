#include <pyuipc/diff_sim/parameter_collection.h>
#include <uipc/diff_sim/parameter_collection.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::diff_sim
{
using namespace uipc::diff_sim;

PyParameterCollection::PyParameterCollection(py::module& m)
{
    auto class_ParameterCollection =
        py::class_<ParameterCollection>(m, "ParameterCollection",
                                         R"(ParameterCollection class for managing simulation parameters.)");

    class_ParameterCollection.def("resize",
                                  &ParameterCollection::resize,
                                  py::arg("N"),
                                  py::arg("default_value") = 0.0,
                                  R"(Resize the parameter collection.
Args:
    N: New size.
    default_value: Default value for new parameters (default: 0.0).)");

    class_ParameterCollection.def("broadcast", &ParameterCollection::broadcast,
                                   R"(Broadcast parameter values across all instances.)");

    class_ParameterCollection.def("view",
                                  [](ParameterCollection& self) {
                                      return as_numpy(self.view(), py::cast(self));
                                  },
                                  R"(Get a view of the parameters as a numpy array.
Returns:
    numpy.ndarray: Array view of parameters.)");

    //m.def(
    //    "view",
    //    [](ParameterCollection& pc) { return as_numpy(view(pc), py::cast(pc)); },
    //    py::arg("pc"));

    top_module().def(
        "view",
        [](ParameterCollection& pc) { return as_numpy(view(pc), py::cast(pc)); },
        py::arg("pc").noconvert(),
        R"(Get a view of parameter collection as a numpy array.
Args:
    pc: ParameterCollection to view.
Returns:
    numpy.ndarray: Array view of parameters.)");
}
}  // namespace pyuipc::diff_sim
