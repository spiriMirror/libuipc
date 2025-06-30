#include <pyuipc/core/feature_collection.h>
#include <uipc/core/feature_collection.h>
#include <pyuipc/common/json.h>
namespace pyuipc::core
{
using namespace uipc::core;
PyFeatureCollection::PyFeatureCollection(py::module& m)
{
    auto class_FeatureCollection = py::class_<FeatureCollection>(m, "FeatureCollection");
    class_FeatureCollection.def(
        "find",
        [](FeatureCollection& self, std::string_view name) -> S<IFeature>
        { return self.find(name); },
        py::return_value_policy::reference_internal);

    class_FeatureCollection.def(
        "find",
        [](FeatureCollection& self, py::type t) -> S<IFeature>
        {
            PYUIPC_ASSERT(!t.attr("FeatureName").is_none(), "Type must be IFeature");
            auto s = t.attr("FeatureName").cast<std::string>();
            return self.find(s);
        },
        py::return_value_policy::reference_internal);

    class_FeatureCollection.def("to_json", &FeatureCollection::to_json);
    class_FeatureCollection.def("__repr__",
                                [&](const FeatureCollection& self)
                                { return self.to_json().dump(4); });
}
}  // namespace pyuipc::core