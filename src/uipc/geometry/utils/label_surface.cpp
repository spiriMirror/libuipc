#include <uipc/geometry/utils/label_surface.h>
#include <uipc/common/timer.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/algorithm/run_length_encode.h>

namespace uipc::geometry
{
P<AttributeSlot<IndexT>> label_surface_vertices(SimplicialComplex& sc)
{
    Timer timer{__FUNCTION__};

    auto is_surf = sc.vertices().find<IndexT>("is_surf");
    if(!is_surf)
    {
        is_surf = sc.vertices().create<IndexT>("is_surf", 0);
    }

    auto IsSurf = geometry::view(*is_surf);

    // if the mesh is 2D, all vertices are on the surface

    if(sc.dim() <= 2)
    {
        std::ranges::fill(IsSurf, 1);
        return is_surf;
    }

    // if the mesh is 3D, we need to find the surface vertices

    // 1) get tetrahedra
    auto Ts = sc.tetrahedra().topo().view();

    // 2) setup separated triangles
    vector<Vector3i> separated_triangles(Ts.size() * 4);

    auto sort_triangle = [](Vector3i T)
    {
        std::sort(T.begin(), T.end());
        return T;
    };

    for(auto&& [i, T] : enumerate(Ts))
    {
        separated_triangles[4 * i + 0] = sort_triangle(Vector3i{T[0], T[1], T[2]});
        separated_triangles[4 * i + 1] = sort_triangle(Vector3i{T[0], T[1], T[3]});
        separated_triangles[4 * i + 2] = sort_triangle(Vector3i{T[0], T[2], T[3]});
        separated_triangles[4 * i + 3] = sort_triangle(Vector3i{T[1], T[2], T[3]});
    }

    // 3) run length encoding the triangles
    std::sort(separated_triangles.begin(),
              separated_triangles.end(),
              [](const Vector3i& a, const Vector3i& b)
              {
                  return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1])
                         || (a[0] == b[0] && a[1] == b[1] && a[2] < b[2]);
              });

    vector<Vector3i> unique_triangles;
    vector<IndexT>   counts;
    unique_triangles.reserve(separated_triangles.size());
    counts.reserve(separated_triangles.size());

    auto unique_count = run_length_encode(separated_triangles.begin(),
                                          separated_triangles.end(),
                                          std::back_inserter(unique_triangles),
                                          std::back_inserter(counts));

    // 4) label the surface vertices and triangles
    for(auto&& [i, F] : enumerate(unique_triangles))
    {
        if(counts[i] == 1)
        {
            IsSurf[F[0]] = 1;
            IsSurf[F[1]] = 1;
            IsSurf[F[2]] = 1;
        }
    }

    return is_surf;
}
}  // namespace uipc::geometry
