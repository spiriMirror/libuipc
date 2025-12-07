#include <pyuipc/core/scene.h>
#include <uipc/core/scene.h>
#include <pyuipc/common/json.h>
#include <uipc/geometry/geometry.h>
#include <pyuipc/common/json.h>
#include <uipc/geometry/attribute_friend.h>

namespace uipc::geometry
{
namespace py = pybind11;
template <>
class AttributeFriend<pyuipc::core::PyScene>
{
  public:
    static S<IAttributeSlot> find(core::Scene::ConfigAttributes& a, std::string_view name)
    {
        return a.m_attributes.find(name);
    }

    static void share(core::Scene::ConfigAttributes& a, std::string_view name, IAttributeSlot& b)
    {
        a.m_attributes.share(name, b);
    }

    static S<IAttributeSlot> create(core::Scene::ConfigAttributes& a,
                                    std::string_view               name,
                                    py::object                     object)
    {
        auto pyobj = py::cast(a.m_attributes,
                              py::return_value_policy::reference_internal,  // member object is a reference in the parent object
                              py::cast(a)  // parent object
        );

        // call the create method of the member object
        return py::cast<S<IAttributeSlot>>(
            pyobj.attr("create").operator()(py::cast(name), object));
    }
};
}  // namespace uipc::geometry

namespace pyuipc::core
{
using namespace uipc::core;

using Accessor = uipc::geometry::AttributeFriend<PyScene>;

void def_method(py::module& m, py::class_<Scene::ConfigAttributes>& class_Attribute)
{
    using Attributes = Scene::ConfigAttributes;

    class_Attribute.def("find",
                        [](Attributes& self, std::string_view name)
                        { return Accessor::find(self, name); },
                        py::arg("name"),
                        R"(Find an attribute by name.
Args:
    name: Name of the attribute to find.
Returns:
    AttributeSlot or None: The attribute slot if found, None otherwise.)");

    class_Attribute.def("destroy",
                        [](Attributes& self, std::string_view name)
                        { std::move(self).destroy(name); },
                        py::arg("name"),
                        R"(Destroy an attribute by name.
Args:
    name: Name of the attribute to destroy.)");

    class_Attribute.def("share",
                        [](Attributes& self, std::string_view name, uipc::geometry::IAttributeSlot& attribute)
                        { Accessor::share(self, name, attribute); },
                        py::arg("name"),
                        py::arg("attribute"),
                        R"(Share an existing attribute slot with a new name.
Args:
    name: New name for the shared attribute.
    attribute: Attribute slot to share.)");

    class_Attribute.def("create",
                        [](Attributes& self, std::string_view name, py::object object)
                        { return Accessor::create(self, name, object); },
                        py::arg("name"),
                        py::arg("object"),
                        R"(Create a new attribute from a Python object.
Args:
    name: Name for the new attribute.
    object: Python object to create attribute from (can be scalar, array, or numpy array).
Returns:
    AttributeSlot: The created attribute slot.)");

    class_Attribute.def("to_json", &Attributes::to_json,
                      R"(Convert attributes to JSON representation.
Returns:
    dict: JSON dictionary representation of the attributes.)");

    class_Attribute.def("__repr__",
                        [](const Attributes& self)
                        { return fmt::format("{}", self.to_json().dump(4)); },
                        R"(String representation of the attributes.
Returns:
    str: Formatted JSON string.)");
}

}  // namespace pyuipc::core

namespace pyuipc::core
{
using namespace uipc::core;
using namespace uipc::geometry;
PyScene::PyScene(py::module& m)
{
    // def class
    auto class_Scene   = py::class_<Scene, S<Scene>>(m, "Scene",
                                                      R"(Scene class representing a simulation scene containing objects, geometries, and constitutions.)");
    auto class_Objects = py::class_<Scene::Objects>(class_Scene, "Objects",
                                                     R"(Collection of objects in the scene.)");
    auto class_Geometries = py::class_<Scene::Geometries>(class_Scene, "Geometries",
                                                          R"(Collection of geometries in the scene.)");


    // def methods
    class_Scene.def(py::init<const Json&>(), py::arg("config") = Scene::default_config(),
                    R"(Create a new scene.
Args:
    config: Configuration dictionary (optional, uses default if not provided).)");

    auto class_ConfigAttributes =
        py::class_<Scene::ConfigAttributes>(class_Scene, "ConfigAttributes",
                                            R"(Configuration attributes for the scene.)");

    def_method(m, class_ConfigAttributes);

    class_Scene.def("config", [](Scene& self) { return self.config(); },
                    R"(Get the scene configuration.
Returns:
    dict: Configuration dictionary.)");

    class_Scene.def_static("default_config", &Scene::default_config,
                          R"(Get the default scene configuration.
Returns:
    dict: Default configuration dictionary.)");

    class_Scene.def(
        "objects",  //
        [](Scene& self) { return self.objects(); },
        py::return_value_policy::move,
        R"(Get the objects collection.
Returns:
    Objects: Collection of objects in the scene.)");
    class_Scene.def(
        "geometries",  //
        [](Scene& self) { return self.geometries(); },
        py::return_value_policy::move,
        R"(Get the geometries collection.
Returns:
    Geometries: Collection of geometries in the scene.)");

    class_Scene.def(
        "contact_tabular",
        [](Scene& self) -> ContactTabular& { return self.contact_tabular(); },
        py::return_value_policy::reference_internal,
        R"(Get the contact tabular (contact system configuration).
Returns:
    ContactTabular: Reference to the contact tabular.)");

    class_Scene.def(
        "subscene_tabular",
        [](Scene& self) -> SubsceneTabular& { return self.subscene_tabular(); },
        py::return_value_policy::reference_internal,
        R"(Get the subscene tabular (subscene configuration).
Returns:
    SubsceneTabular: Reference to the subscene tabular.)");

    class_Scene.def(
        "constitution_tabular",
        [](Scene& self) -> ConstitutionTabular&
        { return self.constitution_tabular(); },
        py::return_value_policy::reference_internal,
        R"(Get the constitution tabular (constitution configuration).
Returns:
    ConstitutionTabular: Reference to the constitution tabular.)");

    class_Scene.def(
        "animator",
        [](Scene& self) -> Animator& { return self.animator(); },
        py::return_value_policy::reference_internal,
        R"(Get the animator for the scene.
Returns:
    Animator: Reference to the scene animator.)");

    class_Objects.def("create",
                      [](Scene::Objects& self, std::string_view name) -> S<Object>
                      { return std::move(self).create(name); },
                      py::arg("name"),
                      R"(Create a new object with the given name.
Args:
    name: Name of the object to create.
Returns:
    Object: The created object.)");

    class_Objects.def("find",
                      [](Scene::Objects& self, IndexT id)
                      { return std::move(self).find(id); },
                      py::arg("id"),
                      R"(Find an object by ID.
Args:
    id: Object ID.
Returns:
    Object or None: The object if found, None otherwise.)");

    class_Objects.def("find",
                      [](Scene::Objects& self, std::string_view name) -> py::list
                      {
                          py::list ret;
                          auto     objects = std::move(self).find(name);
                          for(auto& obj : objects)
                          {
                              ret.append(obj);
                          }
                          return ret;
                      },
                      py::arg("name"),
                      R"(Find all objects with the given name.
Args:
    name: Object name.
Returns:
    list: List of objects with the given name.)");

    class_Objects.def("destroy",
                      [](Scene::Objects& self, IndexT id)
                      { return std::move(self).destroy(id); },
                      py::arg("id"),
                      R"(Destroy an object by ID.
Args:
    id: Object ID to destroy.)");

    class_Objects.def("size",
                      [](Scene::Objects& self) { return std::move(self).size(); },
                      R"(Get the number of objects in the collection.
Returns:
    int: Number of objects.)");

    class_Geometries.def("find",
                         [](Scene::Geometries& self, IndexT id)
                         {
                             auto [geo, rest_geo] = std::move(self).find(id);
                             return std::make_pair(geo, rest_geo);
                         },
                         py::arg("id"),
                         R"(Find geometry and rest geometry by ID.
Args:
    id: Geometry ID.
Returns:
    tuple: Pair of (geometry, rest_geometry).)");

    class_Scene.def(
        "diff_sim",
        [](Scene& self) -> DiffSim& { return self.diff_sim(); },
        py::return_value_policy::reference_internal,
        R"(Get the differential simulator for the scene.
Returns:
    DiffSim: Reference to the differential simulator.)");

    class_Scene.def("__repr__",
                    [](const Scene& self) { return fmt::format("{}", self); },
                    R"(String representation of the scene.
Returns:
    str: String representation.)");
}
}  // namespace pyuipc::core
