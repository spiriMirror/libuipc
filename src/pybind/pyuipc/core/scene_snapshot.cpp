#include <pyuipc/core/scene_snapshot.h>
#include <uipc/core/scene_snapshot.h>
namespace pyuipc::core
{
using namespace uipc::core;
PySceneSnapshot::PySceneSnapshot(py::module& m)
{
    auto class_SceneSnapshot = py::class_<SceneSnapshot>(m, "SceneSnapshot");
    class_SceneSnapshot.def(py::init<const Scene&>(), py::arg("scene"));

    auto class_SceneSnapshotCommit =
        py::class_<SceneSnapshotCommit>(m, "SceneSnapshotCommit");
    class_SceneSnapshotCommit.def(py::init<const SceneSnapshot&, const SceneSnapshot&>(),
                                  py::arg("dst"),
                                  py::arg("src"));

    // __sub__ operator
    class_SceneSnapshot.def("__sub__",
                            [](const SceneSnapshot& dst, const SceneSnapshot& src)
                            { return dst - src; });
}
}  // namespace pyuipc::core
