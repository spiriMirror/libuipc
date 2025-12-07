#include <pyuipc/core/scene_snapshot.h>
#include <uipc/core/scene_snapshot.h>
namespace pyuipc::core
{
using namespace uipc::core;
PySceneSnapshot::PySceneSnapshot(py::module& m)
{
    auto class_SceneSnapshot = py::class_<SceneSnapshot>(m, "SceneSnapshot",
                                                          R"(SceneSnapshot class representing a snapshot of scene state.)");
    class_SceneSnapshot.def(py::init<const Scene&>(), py::arg("scene"),
                            R"(Create a snapshot from a scene.
Args:
    scene: Scene to snapshot.)");

    auto class_SceneSnapshotCommit =
        py::class_<SceneSnapshotCommit>(m, "SceneSnapshotCommit",
                                          R"(SceneSnapshotCommit class representing changes between two scene snapshots.)");
    class_SceneSnapshotCommit.def(py::init<const SceneSnapshot&, const SceneSnapshot&>(),
                                  py::arg("dst"),
                                  py::arg("src"),
                                  R"(Create a commit from two snapshots.
Args:
    dst: Destination snapshot.
    src: Source snapshot.)");

    // __sub__ operator
    class_SceneSnapshot.def("__sub__",
                            [](const SceneSnapshot& dst, const SceneSnapshot& src)
                            { return dst - src; },
                            py::arg("other"),
                            R"(Compute the difference between two snapshots (commit).
Args:
    other: Other snapshot to compare with.
Returns:
    SceneSnapshotCommit: Commit representing the difference.)");
}
}  // namespace pyuipc::core
