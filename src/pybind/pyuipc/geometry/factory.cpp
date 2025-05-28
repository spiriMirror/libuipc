#include <pyuipc/geometry/factory.h>
#include <uipc/geometry/utils/factory.h>
#include <pyuipc/as_numpy.h>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

PyFactory::PyFactory(py::module& m)
{
    m.def(
        "tetmesh",
        [](py::array_t<Float> Vs, py::array_t<IndexT> Ts)
        { return tetmesh(as_span_of<Vector3>(Vs), as_span_of<Vector4i>(Ts)); },
        py::arg("Vs"),
        py::arg("Ts"));

    m.def(
        "trimesh",
        [](py::array_t<Float> Vs, py::array_t<IndexT> Fs) -> SimplicialComplex
        {
            if(is_span_of<Vector3i>(Fs))
                return trimesh(as_span_of<Vector3>(Vs), as_span_of<Vector3i>(Fs));

            if(is_span_of<Vector4i>(Fs))
                return trimesh(as_span_of<Vector3>(Vs), as_span_of<Vector4i>(Fs));

            throw PyException("Invalid face type. Expected Vector4i or Vector3i.");
        },
        py::arg("Vs"),
        py::arg("Fs"));

    m.def(
        "linemesh",
        [](py::array_t<Float> Vs, py::array_t<IndexT> Es)
        { return linemesh(as_span_of<Vector3>(Vs), as_span_of<Vector2i>(Es)); },
        py::arg("Vs"),
        py::arg("Fs"));

    m.def(
        "pointcloud",
        [](py::array_t<Float> Vs) { return pointcloud(as_span_of<Vector3>(Vs)); },
        py::arg("Vs"));

    Vector3 UnitY = Vector3::UnitY();

    m.def(
        "ground",
        [](Float height, py::array_t<Float> N)
        { return ground(height, to_matrix<Vector3>(N)); },
        py::arg("height") = Float{0.0},
        py::arg("N")      = as_numpy(UnitY));

    m.def(
        "halfplane",
        [](py::array_t<Float> P, py::array_t<Float> N)
        { return halfplane(to_matrix<Vector3>(P), to_matrix<Vector3>(N)); },
        py::arg("P") = as_numpy(Vector3::Zero().eval()),
        py::arg("N") = as_numpy(UnitY));
}
}  // namespace pyuipc::geometry
