#include <pyuipc/core/feature.h>
#include <uipc/core/feature.h>
#include <uipc/common/smart_pointer.h>

namespace pyuipc::core
{
using namespace uipc::core;
PyFeature::PyFeature(py::module& m)
{
    auto class_IFeature = py::class_<IFeature, S<IFeature>>(m, "Feature",
                                                             R"(Feature interface for extensible engine functionality.)");
    class_IFeature.def("name",
                       [](const IFeature& self) -> std::string_view
                       { return self.name(); },
                       R"(Get the feature name.
Returns:
    str: Feature name.)");

    class_IFeature.def("type_name",
                       [](const IFeature& self) -> std::string_view
                       { return self.type_name(); },
                       R"(Get the feature type name.
Returns:
    str: Feature type name.)");
}
}  // namespace pyuipc::core
