#include <pyuipc/geometry/geometry_atlas.h>
#include <uipc/geometry/geometry_atlas.h>
#include <pyuipc/common/json.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyGeometryAtlas::PyGeometryAtlas(py::module& m)
{
    auto class_GeometryAtlas = py::class_<GeometryAtlas>(m, "GeometryAtlas");
    class_GeometryAtlas.def(py::init<>());
    class_GeometryAtlas.def(
        "create",
        [](GeometryAtlas& self, Geometry& geo) { return self.create(geo); },
        py::arg("geo"));

    class_GeometryAtlas.def(
        "create",
        [](GeometryAtlas& self, std::string_view name, AttributeCollection& ac)
        { return self.create(name, ac); },
        py::arg("name"),
        py::arg("ac"));

    class_GeometryAtlas.def("geometry_count", &GeometryAtlas::geometry_count);
    class_GeometryAtlas.def("attribute_collection_count",
                            &GeometryAtlas::attribute_collection_count);
    class_GeometryAtlas.def("attribute_collection_names",
                            [](GeometryAtlas& self) -> py::list
                            {
                                auto names = self.attribute_collection_names();
                                py::list py_names;
                                for(auto&& name : names)
                                {
                                    py_names.append(name);
                                }
                                return py_names;
                            });
    class_GeometryAtlas.def("to_json", &GeometryAtlas::to_json);
    class_GeometryAtlas.def("from_json", &GeometryAtlas::from_json);
}
}  // namespace pyuipc::geometry
