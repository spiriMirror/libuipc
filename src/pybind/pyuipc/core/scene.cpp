#include <pyuipc/core/scene.h>
#include <uipc/core/scene.h>
#include <pyuipc/common/json.h>

namespace pyuipc::core
{
using namespace uipc::core;
using namespace uipc::geometry;
PyScene::PyScene(py::module& m)
{
    // def class
    auto class_Scene   = py::class_<Scene>(m, "Scene");
    auto class_Objects = py::class_<Scene::Objects>(class_Scene, "Objects");
    auto class_Geometries = py::class_<Scene::Geometries>(class_Scene, "Geometries");


    // def methods
    class_Scene.def(py::init<const Json&>(), py::arg("config") = Scene::default_config());

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
}
}  // namespace pyuipc::core
