#include <pyuipc/world/scene_io.h>
#include <uipc/util/io/scene_io.h>
namespace pyuipc::world
{
using namespace uipc::world;
PySceneIO::PySceneIO(py::module& m)
{
    auto class_SceneIO = py::class_<SceneIO>(m, "SceneIO");
    class_SceneIO.def(py::init<Scene&>(), py::arg("scene"));
    class_SceneIO.def("write_surface", &SceneIO::write_surface, py::arg("filename"));
    class_SceneIO.def("surface", &SceneIO::surface);
}
}  // namespace pyuipc::world
