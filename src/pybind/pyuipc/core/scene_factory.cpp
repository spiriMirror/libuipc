#include <pyuipc/core/scene_factory.h>
#include <uipc/core/scene_factory.h>
#include <pyuipc/common/json.h>

namespace pyuipc::core
{
using namespace uipc::core;
using namespace uipc::geometry;

PySceneFactory::PySceneFactory(py::module& m)
{
    auto class_SceneFactory = py::class_<SceneFactory>(m, "SceneFactory")
                                  .def(py::init<>())
                                  .def("from_json", &SceneFactory::from_json)
                                  .def("to_json", &SceneFactory::to_json);
}
}  // namespace pyuipc::core
