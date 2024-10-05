#include <pyuipc/geometry/factory.h>
#include <uipc/geometry/utils/factory.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyFactory::PyFactory(py::module& m)
{
    m.def("tetmesh",
          [](py::array_t<Float> Vs, py::array_t<IndexT> Ts)
          { return tetmesh(as_span_of<Vector3>(Vs), as_span_of<Vector4i>(Ts)); });

    m.def("trimesh",
          [](py::array_t<Float> Vs, py::array_t<IndexT> Fs)
          { return trimesh(as_span_of<Vector3>(Vs), as_span_of<Vector3i>(Fs)); });

    m.def("linemesh",
          [](py::array_t<Float> Vs, py::array_t<IndexT> Es)
          { return linemesh(as_span_of<Vector3>(Vs), as_span_of<Vector2i>(Es)); });

    m.def("pointcloud",
          [](py::array_t<Float> Vs)
          { return pointcloud(as_span_of<Vector3>(Vs)); });

    Vector3 UnitY = Vector3::UnitY();

    m.def(
        "ground",
        [](Float height, py::array_t<Float> N)
        { return ground(height, to_matrix<Vector3>(N)); },
        py::arg("height") = Float{0.0},
        py::arg("N")      = as_numpy(UnitY));
}
}  // namespace pyuipc::geometry
