#include <pyuipc/core/scene_io.h>
#include <pyuipc/common/json.h>
#include <uipc/io/scene_io.h>
#include <uipc/core/scene_snapshot.h>

namespace pyuipc::core
{
using namespace uipc::core;
PySceneIO::PySceneIO(py::module& m)
{
    auto class_SceneIO = py::class_<SceneIO>(m, "SceneIO",
                                              R"(SceneIO class for loading and saving scenes to/from files and JSON.)");
    class_SceneIO.def(py::init<Scene&>(), py::arg("scene"),
                     R"(Create a SceneIO instance for a scene.
Args:
    scene: Scene to perform I/O operations on.)");
    class_SceneIO.def("write_surface", &SceneIO::write_surface, py::arg("filename"),
                     R"(Write surface geometry to a file.
Args:
    filename: Output file path.)");
    class_SceneIO.def(
        "simplicial_surface",
        [](SceneIO& self, IndexT dim) { return self.simplicial_surface(dim); },
        py::arg("dim") = -1,
        R"(Get simplicial surface geometry.
Args:
    dim: Dimension of simplices to extract (-1 for all dimensions).
Returns:
    SimplicialComplex: Simplicial surface geometry.)");
    class_SceneIO.def_static(
        "load",
        [](std::string_view filename) { return SceneIO::load(filename); },
        py::arg("filename"),
        R"(Load a scene from a file.
Args:
    filename: Input file path.
Returns:
    Scene: Loaded scene.)");
    class_SceneIO.def(
        "save", [](SceneIO& self, std::string_view file) { self.save(file); }, py::arg("filename"),
        R"(Save the scene to a file.
Args:
    filename: Output file path.)");
    class_SceneIO.def("to_json", &SceneIO::to_json,
                     R"(Convert scene to JSON representation.
Returns:
    dict: JSON representation of the scene.)");
    class_SceneIO.def_static("from_json", &SceneIO::from_json, py::arg("json"),
                            R"(Create a scene from JSON.
Args:
    json: JSON dictionary representing the scene.
Returns:
    Scene: Scene created from JSON.)");

    class_SceneIO.def(
        "commit",
        [](SceneIO& self, const SceneSnapshot& last, std::string_view filename)
        { return self.commit(last, filename); },
        py::arg("last"),
        py::arg("name"),
        R"(Commit scene changes to a file.
Args:
    last: Last scene snapshot for comparison.
    name: Output file path.
Returns:
    SceneSnapshot: New scene snapshot after commit.)");

    class_SceneIO.def(
        "update",
        [](SceneIO& self, std::string_view filename)
        { return self.update(filename); },
        py::arg("filename"),
        R"(Update scene from a file.
Args:
    filename: Input file path.
Returns:
    SceneSnapshot: Scene snapshot after update.)");

    class_SceneIO.def(
        "update_from_json",
        [](SceneIO& self, const Json& json)
        { return self.update_from_json(json); },
        py::arg("commit_json"),
        R"(Update scene from JSON.
Args:
    commit_json: JSON dictionary containing scene updates.
Returns:
    SceneSnapshot: Scene snapshot after update.)");

    class_SceneIO.def(
        "commit_to_json",
        [](SceneIO& self, const SceneSnapshot& reference)
        { return self.commit_to_json(reference); },
        py::arg("reference"),
        R"(Commit scene changes to JSON.
Args:
    reference: Reference scene snapshot for comparison.
Returns:
    dict: JSON dictionary containing committed changes.)");
}
}  // namespace pyuipc::core
