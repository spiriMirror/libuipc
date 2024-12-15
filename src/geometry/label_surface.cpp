#include <uipc/geometry/utils/label_surface.h>
#include <uipc/common/timer.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/algorithm/run_length_encode.h>
#include <uipc/builtin/attribute_name.h>
#include <algorithm>
#include <numeric>

namespace uipc::geometry
{
void label_surface(SimplicialComplex& R)
{
    auto v_is_surf = R.vertices().find<IndexT>(builtin::is_surf);
    if(!v_is_surf)
    {
        v_is_surf = R.vertices().create<IndexT>(builtin::is_surf, 0);
    }

    // if the mesh is a point cloud, all vertices are on the surface
    if(R.dim() == 0)
    {
        std::ranges::fill(view(*v_is_surf), 1);
        return;
    }

    // ----------------------------------------------------------------------------

    auto e_is_surf = R.edges().find<IndexT>(builtin::is_surf);
    if(!e_is_surf)
    {
        e_is_surf = R.edges().create<IndexT>(builtin::is_surf, 0);
    }

    // if the mesh is a line mesh:
    // 1. all vertices are on the surface
    // 2. all edges_with_flag are surface edges_with_flag

    if(R.dim() == 1)
    {
        std::ranges::fill(view(*v_is_surf), 1);
        std::ranges::fill(view(*e_is_surf), 1);
        return;
    }

    // ----------------------------------------------------------------------------

    auto f_is_surf = R.triangles().find<IndexT>(builtin::is_surf);
    if(!f_is_surf)
    {
        f_is_surf = R.triangles().create<IndexT>(builtin::is_surf, 0);
    }


    // if the mesh is 2D:
    // 1. all vertices are on the surface
    // 2. all edges_with_flag are surface edges_with_flag
    // 3. all triangles are surface triangles

    if(R.dim() == 2)
    {
        std::ranges::fill(view(*v_is_surf), 1);
        std::ranges::fill(view(*e_is_surf), 1);
        std::ranges::fill(view(*f_is_surf), 1);
        return;
    }

    // ----------------------------------------------------------------------------

    // if the mesh is 3D, we need to find the surface triangles

    // 1) setup separated triangles
    auto Ts = R.tetrahedra().topo().view();

    auto f_parent_id = R.triangles().find<IndexT>(builtin::parent_id);
    if(!f_parent_id)
    {
        f_parent_id = R.triangles().create<IndexT>(builtin::parent_id, -1);
    }

    vector<Vector4i> separated_triangles(Ts.size() * 4);

    auto sort_triangle = [](const Vector4i& T)
    {
        Vector3i t = T.segment<3>(0);
        std::ranges::sort(t);
        return Vector4i{t[0], t[1], t[2], T[3]};  // the last component is the Tetrahedron index
    };


    for(auto&& [i, T] : enumerate(Ts))
    {
        IndexT I = static_cast<IndexT>(i);

        separated_triangles[4 * i + 0] = sort_triangle(Vector4i{T[0], T[1], T[2], I});
        separated_triangles[4 * i + 1] = sort_triangle(Vector4i{T[0], T[1], T[3], I});
        separated_triangles[4 * i + 2] = sort_triangle(Vector4i{T[0], T[2], T[3], I});
        separated_triangles[4 * i + 3] = sort_triangle(Vector4i{T[1], T[2], T[3], I});
    }

    // 2) run length encoding the triangles
    std::ranges::sort(separated_triangles,
                      [](const Vector4i& a, const Vector4i& b)
                      {
                          return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1])
                                 || (a[0] == b[0] && a[1] == b[1] && a[2] < b[2]);
                      });

    vector<Vector4i> unique_triangles;
    vector<IndexT>   counts;
    unique_triangles.reserve(separated_triangles.size());
    counts.reserve(separated_triangles.size());

    auto unique_count =
        run_length_encode(separated_triangles.begin(),
                          separated_triangles.end(),
                          std::back_inserter(unique_triangles),
                          std::back_inserter(counts),
                          [](const Vector4i& a, const Vector4i& b)
                          { return a.segment<3>(0) == b.segment<3>(0); });

    // Principle:
    // if a triangle is unique in the separated triangles, it is a surface triangle
    // otherwise, it is an internal triangle, because it is shared by two tetrahedra.
    auto is_surface_triangle = [&counts](IndexT i) { return counts[i] == 1; };

    // 3) label the surface tetrahedra
    auto t_is_surf = R.tetrahedra().find<IndexT>(builtin::is_surf);
    if(!t_is_surf)
    {
        t_is_surf = R.tetrahedra().create<IndexT>(builtin::is_surf, 0);
    }
    auto t_is_surf_view = geometry::view(*t_is_surf);

    for(auto&& [i, UF] : enumerate(unique_triangles))
    {
        if(is_surface_triangle(i))
        {
            t_is_surf_view[UF[3]] = 1;
        }
    }

    // 4) label the surface triangles
    auto Fs = R.triangles().topo().view();
    UIPC_ASSERT(f_is_surf->view().size() == unique_triangles.size(),
                "The input mesh should be a closure, why can't we find the same number of triangles? yours {}, ours {}.",
                f_is_surf->view().size(),
                unique_triangles.size());

    {
        auto f_is_surf_view   = view(*f_is_surf);
        auto f_parent_id_view = view(*f_parent_id);
        // now we assume the triangles are sorted
        for(auto&& [i, UF] : enumerate(unique_triangles))
        {
            auto& F      = Fs[i];
            auto  sorted = (F == UF.segment<3>(0));

            // TODO:
            // if the triangles are not sorted, we need find a mapping from the sorted to the unsorted
            // and then we can label the surface vertices
            UIPC_ASSERT(sorted, "The triangles are not sorted, now we don't support this case, TODO: need to implement it.");

            if(is_surface_triangle(i))
            {
                f_is_surf_view[i] = 1;
            }

            f_parent_id_view[i] = UF[3];
        }
    }


    // 5) label the surface edges_with_flag:
    // Principle: if an edge belongs to at least one surface triangle, it is a surface edge.
    auto             Es = R.edges().topo().view();
    vector<Vector3i> edges_with_flag(unique_triangles.size() * 3);
    vector<IndexT>   edge_is_surf(unique_triangles.size() * 3, 0);
    for(auto&& [i, F] : enumerate(unique_triangles))
    {

        IndexT count = counts[i];
        // first 2 components are the edge, the last component is the 'count'
        edges_with_flag[3 * i + 0] = Vector3i{F[0], F[1], count};
        edges_with_flag[3 * i + 1] = Vector3i{F[0], F[2], count};
        edges_with_flag[3 * i + 2] = Vector3i{F[1], F[2], count};
    }

    // sort by the 3 components
    std::ranges::sort(edges_with_flag,
                      [](const Vector3i& a, const Vector3i& b)
                      {
                          return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1])
                                 || (a[0] == b[0] && a[1] == b[1] && a[2] < b[2]);
                          // note that, the last component is the 'count' flag, 1 means the triangle is surface and 2 means the triangle is internal
                          // so if we sort the edges_with_flag by the 3 components, the edges_with_flag have the same vertices [i,j] will be grouped together,
                          // and the smaller count will be in the front.
                      });
    // after sort, we may get:
    // E.g.  ...[a,b,1],[a,b,2],[a,b,2],...,[c,d,1],[c,d,1],...

    // unique on the 3 components: (v0, v1, count)
    auto [first, last] = std::ranges::unique(edges_with_flag);
    edges_with_flag.erase(first, last);
    // after unique, we may get:
    // E.g. ...[a,b,1],[a,b,2],...,[c,d,1],...
    // the edges are still not unique, because the count is different

    // run length encode the edges_with_flag
    vector<Vector3i> unique_edges;
    vector<IndexT>   counts_edges;

    unique_edges.reserve(edges_with_flag.size());
    counts_edges.reserve(edges_with_flag.size());

    // use the first 2 components to run length encode
    auto unique_count_edges =
        run_length_encode(edges_with_flag.begin(),
                          edges_with_flag.end(),
                          std::back_inserter(unique_edges),
                          std::back_inserter(counts_edges),
                          [](const Vector3i& a, const Vector3i& b)
                          { return a.segment<2>(0) == b.segment<2>(0); });

    // exclusive scan to get the offsets, then we can use offset[i] to find the start index of the i-th unique edge
    vector<IndexT> offsets_edges(unique_edges.size());
    std::exclusive_scan(counts_edges.begin(), counts_edges.end(), offsets_edges.begin(), 0);

    //To find the surface edges_with_flag, we only need to check if the edge belongs to a surface triangle
    for(auto e_is_surf_view = view(*e_is_surf); auto&& [i, UE] : enumerate(unique_edges))
    {
        auto& E      = Es[i];
        auto  sorted = (E == UE.segment<2>(0));

        // TODO: if the edges_with_flag are not sorted, we need find a mapping from the sorted to the unsorted
        // 	 and then we can label the surface vertices
        UIPC_ASSERT(sorted, "The edges are not sorted, now we don't support this case, TODO: need to implement it.");

        auto offset         = offsets_edges[i];
        auto edge_with_flag = edges_with_flag[offset];

        if(edge_with_flag(2) == 1)  // count == 1, means the triangle is surface
        {
            e_is_surf_view[i] = 1;
        }
    }

    // 6) label the surface vertices
    for(auto v_is_surf_view = view(*v_is_surf); auto&& [i, F] : enumerate(unique_triangles))
    {
        // vertex is a surface vertex if it is in a surface triangle
        if(is_surface_triangle(i))
        {
            v_is_surf_view[F[0]] = 1;
            v_is_surf_view[F[1]] = 1;
            v_is_surf_view[F[2]] = 1;
        }
    }
}

//SimplicialComplex label_surface(const SimplicialComplex& sc)
//{
//    SimplicialComplex R = sc;
//    label_surface(R);
//    return R;
//}
}  // namespace uipc::geometry
