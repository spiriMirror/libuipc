#include <pyuipc/core/scene_factory.h>
#include <uipc/core/scene_factory.h>
#include <pyuipc/common/json.h>

namespace pyuipc::core
{
using namespace uipc::core;
using namespace uipc::geometry;

PySceneFactory::PySceneFactory(py::module& m)
{
    auto class_SceneFactory = py::class_<SceneFactory>(m, "SceneFactory",
                                                        R"(SceneFactory class for creating scenes from various sources.)")
                                  .def(py::init<>(),
                                       R"(Create a new SceneFactory instance.)")
                                  .def("from_json", &SceneFactory::from_json,
                                       py::arg("json"),
                                       R"(Create a scene from JSON.
Args:
    json: JSON dictionary representing the scene.
Returns:
    Scene: Created scene.)")
                                  .def("to_json", &SceneFactory::to_json,
                                       R"(Convert factory state to JSON.
Returns:
    dict: JSON representation of the factory state.)");

    class_SceneFactory.def("from_snapshot", &SceneFactory::from_snapshot,
                           py::arg("snapshot"),
                           R"(Create a scene from a scene snapshot.
Args:
    snapshot: SceneSnapshot to create scene from.
Returns:
    Scene: Created scene.)");
    class_SceneFactory.def("commit_from_json", &SceneFactory::commit_from_json,
                          py::arg("json"),
                          R"(Apply commit changes from JSON.
Args:
    json: JSON dictionary containing commit changes.)");
    class_SceneFactory.def("commit_to_json", &SceneFactory::commit_to_json,
                           py::arg("reference"),
                           R"(Generate commit JSON from a reference snapshot.
Args:
    reference: Reference SceneSnapshot.
Returns:
    dict: JSON dictionary containing commit changes.)");
}
}  // namespace pyuipc::core
