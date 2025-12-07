#include <pyuipc/core/object.h>
#include <pyuipc/as_numpy.h>
#include <uipc/geometry/simplicial_complex_slot.h>
#include <uipc/geometry/implicit_geometry_slot.h>
#include <uipc/core/object.h>

namespace pyuipc::core
{
using namespace uipc::core;
using namespace uipc::geometry;
PyObject::PyObject(py::module& m)
{
    auto class_Object = py::class_<Object, S<Object>>(m, "Object",
                                                       R"(Object class representing a simulation object containing geometries.)");

    class_Object.def("__repr__",
                     [](const Object& self) { return fmt::format("{}", self); },
                     R"(String representation of the object.
Returns:
    str: String representation.)");

    auto class_Geometries = py::class_<Object::Geometries>(class_Object, "Geometries",
                                                            R"(Collection of geometries belonging to an object.)");

    class_Object.def("name", [](Object& self) { return self.name(); },
                    R"(Get the object name.
Returns:
    str: Object name.)");

    class_Object.def("id", [](Object& self) { return self.id(); },
                    R"(Get the object ID.
Returns:
    int: Object ID.)");

    class_Object.def(
        "geometries", [](Object& self) { return self.geometries(); }, py::return_value_policy::move,
        R"(Get the geometries collection.
Returns:
    Geometries: Collection of geometries.)");

    // For Simplicial Complex

    class_Geometries.def("create",
                         [](Object::Geometries& self, SimplicialComplex& sc)
                         {
                             auto [geo, rest_geo] = std::move(self).create(sc);
                             return std::make_pair(geo, rest_geo);
                         },
                         py::arg("simplicial_complex"),
                         R"(Create geometry from a simplicial complex (rest geometry is auto-generated).
Args:
    simplicial_complex: SimplicialComplex to create geometry from.
Returns:
    tuple: Pair of (geometry, rest_geometry).)");

    class_Geometries.def("create",
                         [](Object::Geometries& self, SimplicialComplex& sc, SimplicialComplex& rest_sc)
                         {
                             auto [geo, rest_geo] = std::move(self).create(sc, rest_sc);
                             return std::make_pair(geo, rest_geo);
                         },
                         py::arg("simplicial_complex"),
                         py::arg("rest_simplicial_complex"),
                         R"(Create geometry from a simplicial complex with explicit rest geometry.
Args:
    simplicial_complex: SimplicialComplex to create geometry from.
    rest_simplicial_complex: Rest state simplicial complex.
Returns:
    tuple: Pair of (geometry, rest_geometry).)");

    // For Implicit Geometry

    class_Geometries.def("create",
                         [](Object::Geometries& self, ImplicitGeometry& ig)
                         {
                             auto [geo, rest_geo] = std::move(self).create(ig);
                             return std::make_pair(geo, rest_geo);
                         },
                         py::arg("implicit_geometry"),
                         R"(Create geometry from an implicit geometry (rest geometry is auto-generated).
Args:
    implicit_geometry: ImplicitGeometry to create geometry from.
Returns:
    tuple: Pair of (geometry, rest_geometry).)");

    class_Geometries.def("create",
                         [](Object::Geometries& self, ImplicitGeometry& ig, ImplicitGeometry& rest_ig)
                         {
                             auto [geo, rest_geo] = std::move(self).create(ig, rest_ig);
                             return std::make_pair(geo, rest_geo);
                         },
                         py::arg("implicit_geometry"),
                         py::arg("rest_implicit_geometry"),
                         R"(Create geometry from an implicit geometry with explicit rest geometry.
Args:
    implicit_geometry: ImplicitGeometry to create geometry from.
    rest_implicit_geometry: Rest state implicit geometry.
Returns:
    tuple: Pair of (geometry, rest_geometry).)");


    // For Geometry
    class_Geometries.def("create",
                         [](Object::Geometries& self, Geometry& geo)
                         {
                             auto [new_geo, rest_geo] = std::move(self).create(geo);
                             return std::make_pair(new_geo, rest_geo);
                         },
                         py::arg("geometry"),
                         R"(Create geometry from an existing geometry (rest geometry is auto-generated).
Args:
    geometry: Geometry to create from.
Returns:
    tuple: Pair of (geometry, rest_geometry).)");

    class_Geometries.def("create",
                         [](Object::Geometries& self, Geometry& geo, Geometry& rest_geo)
                         {
                             auto [new_geo, new_rest_geo] =
                                 std::move(self).create(geo, rest_geo);
                             return std::make_pair(new_geo, new_rest_geo);
                         },
                         py::arg("geometry"),
                         py::arg("rest_geometry"),
                         R"(Create geometry from an existing geometry with explicit rest geometry.
Args:
    geometry: Geometry to create from.
    rest_geometry: Rest state geometry.
Returns:
    tuple: Pair of (geometry, rest_geometry).)");

    class_Geometries.def("ids",
                         [](Object::Geometries& self) {
                             return as_numpy(std::move(self).ids(), py::cast(self));
                         },
                         R"(Get the IDs of all geometries in the collection.
Returns:
    numpy.ndarray: Array of geometry IDs.)");
}
}  // namespace pyuipc::core