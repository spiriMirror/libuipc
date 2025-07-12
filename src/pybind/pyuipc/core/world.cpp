#include <pyuipc/core/world.h>
#include <uipc/core/world.h>
#include <uipc/core/engine.h>

namespace pyuipc::core
{
using namespace uipc::core;

PyWorld::PyWorld(py::module& m)
{
    auto class_World = py::class_<World, S<World>>(m, "World");

    class_World.def(py::init<Engine&>())
        .def("init", &World::init, py::arg("scene"), py::call_guard<py::gil_scoped_release>())
        .def("advance", &World::advance, py::call_guard<py::gil_scoped_release>())
        .def("sync", &World::sync, py::call_guard<py::gil_scoped_release>())
        .def("retrieve", &World::retrieve, py::call_guard<py::gil_scoped_release>())
        .def("dump", &World::dump, py::call_guard<py::gil_scoped_release>())
        .def("recover",
             &World::recover,
             py::arg("dst_frame") = ~0ull,
             py::call_guard<py::gil_scoped_release>())
        .def("backward", &World::backward, py::call_guard<py::gil_scoped_release>())
        .def("frame", &World::frame, py::call_guard<py::gil_scoped_release>())
        .def("features",
             &World::features,
             py::return_value_policy::reference_internal,
             py::call_guard<py::gil_scoped_release>())
        .def("is_valid", &World::is_valid, py::call_guard<py::gil_scoped_release>());
}

}  // namespace pyuipc::core
