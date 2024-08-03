#include <pyuipc/geometry/geometry.h>
#include <uipc/geometry/geometry.h>
#include <pyuipc/common/json.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyGeometry::PyGeometry(py::module& m)
{
    auto class_Geometry = py::class_<Geometry>(m, "Geometry");

    auto class_MetaAttributes =
        py::class_<Geometry::MetaAttributes>(class_Geometry, "MetaAttributes");

    auto class_InstanceAttributes =
        py::class_<Geometry::InstanceAttributes>(class_Geometry, "InstanceAttributes");

    class_Geometry.def("meta", [](Geometry& self) { return self.meta(); });
    class_Geometry.def("instances",
                       [](Geometry& self) { return self.instances(); });

    class_Geometry.def(
        "transforms",
        [](Geometry& self) -> AttributeSlot<Matrix4x4>&
        { return self.transforms(); },
        py::return_value_policy::reference_internal);

    class_Geometry.def("to_json", [](Geometry& self) { return self.to_json(); });
}
}  // namespace pyuipc::geometry
