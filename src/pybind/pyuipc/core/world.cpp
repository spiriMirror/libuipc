#include <pyuipc/core/world.h>
#include <uipc/core/world.h>
#include <uipc/core/engine.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyWorld::PyWorld(py::module& m)
{
    auto class_World = py::class_<World, S<World>>(m, "World",
                                                    R"(World class representing the simulation world that manages scenes and simulation state.)");

    class_World.def(py::init<Engine&>(),
                   py::arg("engine"),
                   R"(Create a world with an engine.
Args:
    engine: Engine instance to use.)");

    class_World.def("init", &World::init, py::arg("scene"), py::call_guard<py::gil_scoped_release>(),
                   R"(Initialize the world with a scene.
Args:
    scene: Scene to initialize the world with.)");
    class_World.def("advance", &World::advance, py::call_guard<py::gil_scoped_release>(),
                   R"(Advance the simulation by one step.)");
    class_World.def("sync", &World::sync, py::call_guard<py::gil_scoped_release>(),
                   R"(Synchronize the world state.)");
    class_World.def("retrieve", &World::retrieve, py::call_guard<py::gil_scoped_release>(),
                   R"(Retrieve the world state.)");
    class_World.def("dump", &World::dump, py::call_guard<py::gil_scoped_release>(),
                   R"(Dump the world state.)");
    class_World.def("recover",
                    &World::recover,
                    py::arg("dst_frame") = ~0ull,
                    py::call_guard<py::gil_scoped_release>(),
                    R"(Recover the world state to a specific frame.
Args:
    dst_frame: Target frame number (default: maximum frame).)");
    class_World.def("frame", &World::frame, py::call_guard<py::gil_scoped_release>(),
                   R"(Get the current frame number.
Returns:
    int: Current frame number.)");
    class_World.def("features",
                    &World::features,
                    py::return_value_policy::reference_internal,
                    py::call_guard<py::gil_scoped_release>(),
                    R"(Get the feature collection.
Returns:
    FeatureCollection: Reference to feature collection.)");
    class_World.def("is_valid", &World::is_valid, py::call_guard<py::gil_scoped_release>(),
                   R"(Check if the world is in a valid state.
Returns:
    bool: True if valid, False otherwise.)");

    class_World.def(
        "sanity_checker",
        [](World& self) -> SanityChecker& { return self.sanity_checker(); },
        py::return_value_policy::reference_internal,
        py::call_guard<py::gil_scoped_release>(),
        R"(Get the sanity checker.
Returns:
    SanityChecker: Reference to sanity checker.)");
}

}  // namespace pyuipc::core
