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
        py::arg("Ts"),
        R"(Create a tetrahedral mesh from vertices and tetrahedra.
Args:
    Vs: Array of vertex positions (Nx3).
    Ts: Array of tetrahedron indices (Mx4).
Returns:
    SimplicialComplex: Tetrahedral mesh.)");

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
        py::arg("Fs"),
        R"(Create a triangular mesh from vertices and faces.
Args:
    Vs: Array of vertex positions (Nx3).
    Fs: Array of face indices (Mx3 for triangles or Mx4 for quads).
Returns:
    SimplicialComplex: Triangular mesh.)");

    m.def(
        "linemesh",
        [](py::array_t<Float> Vs, py::array_t<IndexT> Es)
        { return linemesh(as_span_of<Vector3>(Vs), as_span_of<Vector2i>(Es)); },
        py::arg("Vs"),
        py::arg("Es"),
        R"(Create a line mesh from vertices and edges.
Args:
    Vs: Array of vertex positions (Nx3).
    Es: Array of edge indices (Mx2).
Returns:
    SimplicialComplex: Line mesh.)");
    m.def(
        "pointcloud",
        [](py::array_t<Float> Vs) { return pointcloud(as_span_of<Vector3>(Vs)); },
        py::arg("Vs"),
        R"(Create a point cloud from vertices.
Args:
    Vs: Array of vertex positions (Nx3).
Returns:
    SimplicialComplex: Point cloud.)");

    Vector3 UnitY = Vector3::UnitY();

    m.def(
        "ground",
        [](Float height, py::array_t<Float> N)
        { return ground(height, to_matrix<Vector3>(N)); },
        py::arg("height") = Float{0.0},
        py::arg("N")      = as_numpy(UnitY),
        R"(Create a ground plane implicit geometry.
Args:
    height: Height of the ground plane (default: 0.0).
    N: Normal vector of the ground plane (default: Y-axis).
Returns:
    ImplicitGeometry: Ground plane.)");

    m.def(
        "halfplane",
        [](py::array_t<Float> P, py::array_t<Float> N)
        { return halfplane(to_matrix<Vector3>(P), to_matrix<Vector3>(N)); },
        py::arg("P") = as_numpy(Vector3::Zero().eval()),
        py::arg("N") = as_numpy(UnitY),
        R"(Create a half-plane implicit geometry.
Args:
    P: Point on the plane (default: origin).
    N: Normal vector of the plane (default: Y-axis).
Returns:
    ImplicitGeometry: Half-plane.)");
}
}  // namespace pyuipc::geometry
