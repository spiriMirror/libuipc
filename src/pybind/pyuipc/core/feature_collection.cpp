#include <pyuipc/core/feature_collection.h>
#include <uipc/core/feature_collection.h>
#include <pyuipc/common/json.h>
namespace pyuipc::core
{
using namespace uipc::core;
PyFeatureCollection::PyFeatureCollection(py::module& m)
{
    auto class_FeatureCollection = py::class_<FeatureCollection>(m, "FeatureCollection",
                                                                  R"(FeatureCollection class managing engine features.)");
    class_FeatureCollection.def(
        "find",
        [](FeatureCollection& self, std::string_view name) -> S<IFeature>
        { return self.find(name); },
        py::return_value_policy::reference_internal,
        py::arg("name"),
        R"(Find a feature by name.
Args:
    name: Feature name.
Returns:
    Feature or None: Feature if found, None otherwise.)");

    class_FeatureCollection.def(
        "find",
        [](FeatureCollection& self, py::type t) -> S<IFeature>
        {
            PYUIPC_ASSERT(!t.attr("FeatureName").is_none(), "Type must be IFeature");
            auto s = t.attr("FeatureName").cast<std::string>();
            return self.find(s);
        },
        py::return_value_policy::reference_internal,
        py::arg("type"),
        R"(Find a feature by type.
Args:
    type: Feature type (must have FeatureName attribute).
Returns:
    Feature or None: Feature if found, None otherwise.)");

    class_FeatureCollection.def("to_json", &FeatureCollection::to_json,
                                R"(Convert feature collection to JSON.
Returns:
    dict: JSON representation of the feature collection.)");
    class_FeatureCollection.def("__repr__",
                                [&](const FeatureCollection& self)
                                { return self.to_json().dump(4); },
                                R"(String representation of the feature collection.
Returns:
    str: Formatted JSON string.)");
}
}  // namespace pyuipc::core