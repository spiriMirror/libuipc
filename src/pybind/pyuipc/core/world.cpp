#include <pyuipc/core/world.h>
#include <uipc/core/world.h>
#include <uipc/core/engine.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyWorld::PyWorld(py::module& m)
{
    auto class_World = py::class_<World, S<World>>(m, "World");

    class_World.def(py::init<Engine&>());

    class_World.def("init", &World::init, py::arg("scene"), py::call_guard<py::gil_scoped_release>());
    class_World.def("advance", &World::advance, py::call_guard<py::gil_scoped_release>());
    class_World.def("sync", &World::sync, py::call_guard<py::gil_scoped_release>());
    class_World.def("retrieve", &World::retrieve, py::call_guard<py::gil_scoped_release>());
    class_World.def("dump", &World::dump, py::call_guard<py::gil_scoped_release>());
    class_World.def("recover",
                    &World::recover,
                    py::arg("dst_frame") = ~0ull,
                    py::call_guard<py::gil_scoped_release>());
    class_World.def("frame", &World::frame, py::call_guard<py::gil_scoped_release>());
    class_World.def("features",
                    &World::features,
                    py::return_value_policy::reference_internal,
                    py::call_guard<py::gil_scoped_release>());
    class_World.def("is_valid", &World::is_valid, py::call_guard<py::gil_scoped_release>());

    class_World.def(
        "sanity_checker",
        [](World& self) -> SanityChecker& { return self.sanity_checker(); },
        py::return_value_policy::reference_internal,
        py::call_guard<py::gil_scoped_release>());
}

}  // namespace pyuipc::core
