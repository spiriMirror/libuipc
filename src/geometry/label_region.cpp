#include <uipc/geometry/utils/label_region.h>
#include <uipc/geometry/utils/label_connected_vertices.h>

namespace uipc::geometry
{
void label_region(SimplicialComplex& complex)
{
    // 1) find the region of each vertex
    auto vert_region = complex.vertices().find<IndexT>("region");
    if(!vert_region)  // if not exist, label the connected vertices
        vert_region = label_connected_vertices(complex);

    auto vert_region_view = vert_region->view();

    auto Dim = complex.dim();

    // 2) fill the region of each edge
    if(Dim >= 1)
    {
        auto edge_region = complex.edges().find<IndexT>("region");
        if(!edge_region)
            edge_region = complex.edges().create<IndexT>("region");

        auto edge_region_view = view(*edge_region);
        auto edge_view        = complex.edges().topo().view();

        for(auto [i, edge] : enumerate(edge_view))
        {
            UIPC_ASSERT(vert_region_view[edge[0]] == vert_region_view[edge[1]],
                        "In the edge[{}] = ({},{}), vertices are not in the same region -> ({},{}), which is ill condition.",
                        i,
                        edge[0],
                        edge[1],
                        vert_region_view[edge[0]],
                        vert_region_view[edge[1]]);

            edge_region_view[i] = vert_region_view[edge[0]];
        }
    }

    // 3) fill the region of each triangle
    if(Dim >= 2)
    {
        auto tri_region = complex.triangles().find<IndexT>("region");
        if(!tri_region)
            tri_region = complex.triangles().create<IndexT>("region");

        auto tri_region_view = view(*tri_region);
        auto tri_view        = complex.triangles().topo().view();

        for(auto [i, tri] : enumerate(tri_view))
        {
            UIPC_ASSERT(vert_region_view[tri[0]] == vert_region_view[tri[1]]
                            && vert_region_view[tri[1]] == vert_region_view[tri[2]],
                        "In the triangle[{}] = ({},{},{}), vertices are not in the same region -> ({},{},{}), which is ill condition.",
                        i,
                        tri[0],
                        tri[1],
                        tri[2],
                        vert_region_view[tri[0]],
                        vert_region_view[tri[1]],
                        vert_region_view[tri[2]]);

            tri_region_view[i] = vert_region_view[tri[0]];
        }
    }

    // 4) fill the region of each tetrahedron
    if(Dim >= 3)
    {
        auto tet_region = complex.tetrahedra().find<IndexT>("region");
        if(!tet_region)
            tet_region = complex.tetrahedra().create<IndexT>("region");

        auto tet_region_view = view(*tet_region);
        auto tet_view        = complex.tetrahedra().topo().view();

        for(auto [i, tet] : enumerate(tet_view))
        {
            UIPC_ASSERT(vert_region_view[tet[0]] == vert_region_view[tet[1]]
                            && vert_region_view[tet[1]] == vert_region_view[tet[2]]
                            && vert_region_view[tet[2]] == vert_region_view[tet[3]],
                        "In the tetrahedron[{}] = ({},{},{},{}), vertices are not in the same region -> ({},{},{},{}), which is ill condition.",
                        i,
                        tet[0],
                        tet[1],
                        tet[2],
                        tet[3],
                        vert_region_view[tet[0]],
                        vert_region_view[tet[1]],
                        vert_region_view[tet[2]],
                        vert_region_view[tet[3]]);

            tet_region_view[i] = vert_region_view[tet[0]];
        }
    }
}
}  // namespace uipc::geometry
