#include <pyuipc/core/scene_io.h>
#include <pyuipc/common/json.h>
#include <uipc/io/scene_io.h>
#include <uipc/core/scene_snapshot.h>

namespace pyuipc::core
{
using namespace uipc::core;
PySceneIO::PySceneIO(py::module& m)
{
    auto class_SceneIO = py::class_<SceneIO>(m, "SceneIO");
    class_SceneIO.def(py::init<Scene&>(), py::arg("scene"));
    class_SceneIO.def("write_surface", &SceneIO::write_surface, py::arg("filename"));
    class_SceneIO.def(
        "simplicial_surface",
        [](SceneIO& self, IndexT dim) { return self.simplicial_surface(dim); },
        py::arg("dim") = -1);
    class_SceneIO.def_static(
        "load",
        [](std::string_view filename) { return SceneIO::load(filename); },
        py::arg("filename"));
    class_SceneIO.def(
        "save", [](SceneIO& self, std::string_view file) { self.save(file); }, py::arg("filename"));
    class_SceneIO.def("to_json", &SceneIO::to_json);
    class_SceneIO.def_static("from_json", &SceneIO::from_json, py::arg("json"));

    class_SceneIO.def(
        "commit",
        [](SceneIO& self, const SceneSnapshot& last, std::string_view filename)
        { return self.commit(last, filename); },
        py::arg("last"),
        py::arg("name"));

    class_SceneIO.def(
        "update",
        [](SceneIO& self, std::string_view filename)
        { return self.update(filename); },
        py::arg("filename"));

    class_SceneIO.def(
        "update_from_json",
        [](SceneIO& self, const Json& json)
        { return self.update_from_json(json); },
        py::arg("commit_json"));

    class_SceneIO.def(
        "commit_to_json",
        [](SceneIO& self, const SceneSnapshot& reference)
        { return self.commit_to_json(reference); },
        py::arg("reference"));
}
}  // namespace pyuipc::core
