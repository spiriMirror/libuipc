#include <uipc/geometry/utils/points_from_volume.h>
#include <uipc/geometry/utils/label_surface.h>
#include <uipc/geometry/utils/extract_surface.h>
#include <uipc/geometry/utils/is_trimesh_closed.h>
#include <Eigen/Core>
#include <igl/winding_number.h>
#include <vector>
#include <tbb/tbb.h>
#include <limits>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/utils/label_triangle_orient.h>
#include <uipc/geometry/utils/factory.h>
#include <uipc/geometry/utils/flip_inward_triangles.h>

namespace uipc::geometry
{
SimplicialComplex scan_trimesh(const SimplicialComplex& sc, Float resolution)
{
    SimplicialComplex R;

    MatrixX         V;
    Eigen::MatrixXi F;
    V.resize(sc.vertices().size(), 3);
    F.resize(sc.triangles().size(), 3);
    auto V_span = sc.positions().view();
    auto F_span = sc.triangles().topo().view();

    for(auto [i, value] : enumerate(V_span))
        V.row(i) = value;
    for(auto [i, value] : enumerate(F_span))
        F.row(i) = value;

    // 1. compute bounding box with padding
    Vector3 min_corner = V.colwise().minCoeff().array() - resolution / 2;
    Vector3 max_corner = V.colwise().maxCoeff().array() + resolution / 2;

    // 2. compute number of voxels in each dimension
    auto nx = static_cast<size_t>((max_corner.x() - min_corner.x()) / resolution) + 1;
    auto ny = static_cast<size_t>((max_corner.y() - min_corner.y()) / resolution) + 1;
    auto nz = static_cast<SizeT>((max_corner.z() - min_corner.z()) / resolution) + 1;

    SizeT voxel_count = nx * ny * nz;

    // 3. parallel for loop to generate points
    tbb::concurrent_vector<Vector3> inside_points;
    inside_points.reserve(voxel_count);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, voxel_count),
                      [&](const tbb::blocked_range<size_t>& r)
                      {
                          for(SizeT i = r.begin(); i < r.end(); ++i)
                          {
                              Eigen::MatrixXd Q(1, 3);
                              SizeT           ix = i % nx;
                              SizeT           iy = (i / nx) % ny;
                              SizeT           iz = i / (nx * ny);
                              Q(0, 0) = min_corner.x() + ix * resolution;
                              Q(0, 1) = min_corner.y() + iy * resolution;
                              Q(0, 2) = min_corner.z() + iz * resolution;
                              Eigen::VectorXd wn;
                              igl::winding_number(V, F, Q, wn);
                              if(wn[0] > 0.5)
                              {
                                  inside_points.push_back(Q.row(0));
                              }
                          }
                      });

    std::vector<Vector3> vertex_indices{inside_points.begin(), inside_points.end()};
    return pointcloud(vertex_indices);
}

SimplicialComplex points_from_volume(const SimplicialComplex& sc, Float resolution)
{
    UIPC_ASSERT(sc.dim() >= 2 && sc.dim() <= 3,
                "points_from_volume: Only 2D and 3D simplicial complexes are supported.");

    if(sc.dim() == 2)
    {
        UIPC_ASSERT(is_trimesh_closed(sc),
                    "points_from_volume: The input 2D simplicial complex must be a closed manifold.");
        return scan_trimesh(sc, resolution);
    }

    if(sc.dim() == 3)
    {
        SimplicialComplex tmp_sc = sc;
        label_surface(tmp_sc);
        label_triangle_orient(tmp_sc);
        auto surface  = extract_surface(tmp_sc);
        auto oriented = flip_inward_triangles(surface);
        return scan_trimesh(oriented, resolution);
    }

    // avoid compiler warning
    return SimplicialComplex{};
}
}  // namespace uipc::geometry