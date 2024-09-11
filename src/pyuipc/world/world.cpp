#include <pyuipc/world/world.h>
#include <uipc/world/world.h>

namespace pyuipc::world
{
using namespace uipc::world;

PyWorld::PyWorld(py::module& m)
{
    auto class_World = py::class_<World>(m, "World");

    class_World
        .def(py::init([](engine::IEngine& e)
                      { return std::make_unique<World>(e); }))
        .def("init", &World::init, py::arg("scene"))
        .def("advance", &World::advance)
        .def("sync", &World::sync)
        .def("retrieve", &World::retrieve)
        .def("dump", &World::dump)
        .def("recover", &World::recover)
        .def("frame", &World::frame)
        .def("animator", &World::animator, py::return_value_policy::reference_internal);
}

}  // namespace pyuipc::world
