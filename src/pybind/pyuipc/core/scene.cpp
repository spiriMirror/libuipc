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
                        { return Accessor::find(self, name); });

    class_Attribute.def("destroy",
                        [](Attributes& self, std::string_view name)
                        { std::move(self).destroy(name); });

    class_Attribute.def("share",
                        [](Attributes& self, std::string_view name, uipc::geometry::IAttributeSlot& attribute)
                        { Accessor::share(self, name, attribute); });

    class_Attribute.def("create",
                        [](Attributes& self, std::string_view name, py::object object)
                        { return Accessor::create(self, name, object); });

    class_Attribute.def("to_json", &Attributes::to_json);
}

}  // namespace pyuipc::core

namespace pyuipc::core
{
using namespace uipc::core;
using namespace uipc::geometry;
PyScene::PyScene(py::module& m)
{
    // def class
    auto class_Scene   = py::class_<Scene, S<Scene>>(m, "Scene");
    auto class_Objects = py::class_<Scene::Objects>(class_Scene, "Objects");
    auto class_Geometries = py::class_<Scene::Geometries>(class_Scene, "Geometries");


    // def methods
    class_Scene.def(py::init<const Json&>(), py::arg("config") = Scene::default_config());

    auto class_ConfigAttributes =
        py::class_<Scene::ConfigAttributes>(class_Scene, "ConfigAttributes");

    def_method(m, class_ConfigAttributes);

    class_Scene.def("config", [](Scene& self) { return self.config(); });

    class_Scene.def_static("default_config", &Scene::default_config);

    class_Scene.def(
        "objects",  //
        [](Scene& self) { return self.objects(); },
        py::return_value_policy::move);
    class_Scene.def(
        "geometries",  //
        [](Scene& self) { return self.geometries(); },
        py::return_value_policy::move);

    class_Scene.def(
        "contact_tabular",
        [](Scene& self) -> ContactTabular& { return self.contact_tabular(); },
        py::return_value_policy::reference_internal);

    class_Scene.def(
        "constitution_tabular",
        [](Scene& self) -> ConstitutionTabular&
        { return self.constitution_tabular(); },
        py::return_value_policy::reference_internal);

    class_Scene.def(
        "animator",
        [](Scene& self) -> Animator& { return self.animator(); },
        py::return_value_policy::reference_internal);

    class_Objects.def("create",
                      [](Scene::Objects& self, std::string_view name) -> S<Object>
                      { return std::move(self).create(name); });

    class_Objects.def("find",
                      [](Scene::Objects& self, IndexT id)
                      { return std::move(self).find(id); });

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
                      });

    class_Objects.def("destroy",
                      [](Scene::Objects& self, IndexT id)
                      { return std::move(self).destroy(id); });

    class_Objects.def("size",
                      [](Scene::Objects& self) { return std::move(self).size(); });

    class_Geometries.def("find",
                         [](Scene::Geometries& self, IndexT id)
                         {
                             auto [geo, rest_geo] = std::move(self).find(id);
                             return std::make_pair(geo, rest_geo);
                         });

    class_Scene.def(
        "diff_sim",
        [](Scene& self) -> DiffSim& { return self.diff_sim(); },
        py::return_value_policy::reference_internal);

    class_Scene.def(
        "sanity_checker",
        [](Scene& self) -> SanityChecker& { return self.sanity_checker(); },
        py::return_value_policy::reference_internal);

    class_Scene.def("__repr__",
                    [](const Scene& self) { return fmt::format("{}", self); });
}
}  // namespace pyuipc::core
