#include <pyuipc/geometry/geometry_atlas.h>
#include <uipc/geometry/geometry_atlas.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyGeometryAtlas::PyGeometryAtlas(py::module& m)
{
    auto class_GeometryAtlas = py::class_<GeometryAtlas>(
        m, "GeometryAtlas", R"(GeometryAtlas class for managing multiple geometries and attribute collections.)");
    class_GeometryAtlas.def(py::init<>(), R"(Create an empty geometry atlas.)");
    class_GeometryAtlas.def(
        "create",
        [](GeometryAtlas& self, Geometry& geo, bool evolving_only)
        { return self.create(geo, evolving_only); },
        py::arg("geo"),
        py::arg("evolving_only") = false,
        R"(Create a geometry in the atlas.
Args:
    geo: Geometry to add.
    evolving_only: Keep only evolving attributes for baseline-dependent streaming.
Returns:
    int: Atlas geometry ID.)");

    class_GeometryAtlas.def(
        "create",
        [](GeometryAtlas& self, std::string_view name, AttributeCollection& ac, bool evolving_only)
        { return self.create(name, ac, evolving_only); },
        py::arg("name"),
        py::arg("ac"),
        py::arg("evolving_only") = false,
        R"(Create a named attribute collection.
Args:
    name: Name for the attribute collection.
    ac: AttributeCollection to copy.
    evolving_only: Keep only evolving attributes while preserving row count.)");

    class_GeometryAtlas.def("geometry_count",
                            &GeometryAtlas::geometry_count,
                            R"(Get the number of geometries in the atlas.
Returns:
    int: Number of geometries.)");
    class_GeometryAtlas.def("attribute_collection_count",
                            &GeometryAtlas::attribute_collection_count,
                            R"(Get the number of attribute collections.
Returns:
    int: Number of attribute collections.)");
    class_GeometryAtlas.def(
        "attribute_collection_names",
        [](GeometryAtlas& self) -> py::list
        {
            auto     names = self.attribute_collection_names();
            py::list py_names;
            for(auto&& name : names)
            {
                py_names.append(name);
            }
            return py_names;
        },
        R"(Get the names of all attribute collections.
Returns:
    list: List of attribute collection names.)");
    class_GeometryAtlas.def("to_json",
                            &GeometryAtlas::to_json,
                            R"(Convert atlas to JSON representation.
Returns:
    dict: JSON representation of the atlas.)");
    class_GeometryAtlas.def("from_json",
                            &GeometryAtlas::from_json,
                            py::arg("json"),
                            R"(Load atlas from JSON representation.
Args:
    json: JSON dictionary to load from.)");
}
}  // namespace pyuipc::geometry
