#include <pyuipc/diff_sim/parameter_collection.h>
#include <uipc/diff_sim/parameter_collection.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::diff_sim
{
using namespace uipc::diff_sim;

PyParameterCollection::PyParameterCollection(py::module& m)
{
    auto class_ParameterCollection =
        py::class_<ParameterCollection>(m, "ParameterCollection");

    class_ParameterCollection.def("resize",
                                  &ParameterCollection::resize,
                                  py::arg("N"),
                                  py::arg("default_value") = 0.0);

    class_ParameterCollection.def("broadcast", &ParameterCollection::broadcast);

    class_ParameterCollection.def("view",
                                  [](ParameterCollection& self) {
                                      return as_numpy(self.view(), py::cast(self));
                                  });

    //m.def(
    //    "view",
    //    [](ParameterCollection& pc) { return as_numpy(view(pc), py::cast(pc)); },
    //    py::arg("pc"));

    top_module().def(
        "view",
        [](ParameterCollection& pc) { return as_numpy(view(pc), py::cast(pc)); },
        py::arg("pc").noconvert());
}
}  // namespace pyuipc::diff_sim
