#include <pyuipc/geometry/factory.h>
#include <uipc/geometry/details/factory.h>
#include <uipc/geometry/utils/closure.h>
#include <uipc/geometry/utils/factory.h>
#include <cstring>

namespace pyuipc::geometry
{
using namespace uipc::geometry;

namespace
{
    template <typename VectorT>
    class ArrayRows
    {
        using Scalar               = typename VectorT::Scalar;
        static constexpr int Width = VectorT::SizeAtCompileTime;

      public:
        explicit ArrayRows(const py::array_t<Scalar>& array)
            : m_array(array)
        {
            if(!is_span_of<VectorT>(array))
                throw PyException(
                    PYUIPC_MSG("Expected array shape (N,{}) or (N,{},1)", Width, Width));
        }

        SizeT size() const { return static_cast<SizeT>(m_array.shape(0)); }

        VectorT operator[](SizeT i) const
        {
            const auto* row = reinterpret_cast<const char*>(m_array.data())
                              + static_cast<py::ssize_t>(i) * m_array.strides(0);
            VectorT value;
            for(int j = 0; j < Width; ++j)
                std::memcpy(&value[j], row + j * m_array.strides(1), sizeof(Scalar));
            return value;
        }

        void copy_to(span<VectorT> dst) const
        {
            if(m_array.flags() & py::array::c_style)
            {
                // Copy the packed scalar rows, including arrays with unaligned data.
                const char* src = reinterpret_cast<const char*>(m_array.data());
                for(auto& v : dst)
                {
                    std::memcpy(v.data(), src, Width * sizeof(Scalar));
                    src += Width * sizeof(Scalar);
                }
                return;
            }
            for(SizeT i = 0; i < dst.size(); ++i)
                dst[i] = (*this)[i];
        }

      private:
        const py::array_t<Scalar>& m_array;
    };

    SimplicialComplex pointcloud_from_array(const py::array_t<Float>& Vs)
    {
        SimplicialComplex  sc;
        ArrayRows<Vector3> vertices{Vs};
        vertices.copy_to(uipc::geometry::detail::resize_positions(sc, vertices.size()));
        return sc;
    }
}  // namespace

PyFactory::PyFactory(py::module& m)
{
    m.def(
        "tetmesh",
        [](py::array_t<Float> Vs, py::array_t<IndexT> Ts)
        {
            auto                sc = pointcloud_from_array(Vs);
            ArrayRows<Vector4i> tets{Ts};
            tets.copy_to(uipc::geometry::detail::resize_topology(sc.tetrahedra(),
                                                                 tets.size()));
            return facet_closure(sc);
        },
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
            auto sc = pointcloud_from_array(Vs);
            if(is_span_of<Vector4i>(Fs))
            {
                ArrayRows<Vector4i> quads{Fs};
                uipc::geometry::detail::triangulate_quads(sc, quads);
            }
            else
            {
                ArrayRows<Vector3i> faces{Fs};
                faces.copy_to(uipc::geometry::detail::resize_topology(sc.triangles(),
                                                                      faces.size()));
            }
            return facet_closure(sc);
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
        {
            auto                sc = pointcloud_from_array(Vs);
            ArrayRows<Vector2i> edges{Es};
            edges.copy_to(uipc::geometry::detail::resize_topology(sc.edges(),
                                                                  edges.size()));
            return facet_closure(sc);
        },
        py::arg("Vs"),
        py::arg("Es"),
        R"(Create a line mesh from vertices and edges.
Args:
    Vs: Array of vertex positions (Nx3).
    Es: Array of edge indices (Mx2).
Returns:
    SimplicialComplex: Line mesh.)");
    m.def("pointcloud",
          &pointcloud_from_array,
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
