#include <uipc/geometry/utils/apply_region.h>
#include <uipc/geometry/utils/label_region.h>
#include <uipc/common/map.h>
namespace uipc::geometry
{
static void calculate_mapping(SizeT                  N_region,
                              span<const IndexT>     region,
                              vector<SizeT>&         G2L,
                              vector<vector<SizeT>>& L2G)
{
    vector<SizeT> region_prim_count(N_region, 0);
    G2L.resize(region.size(), -1);
    L2G.resize(N_region);

    for(auto&& [i, r] : enumerate(region))
    {
        auto& local_prim = region_prim_count[r];
        G2L[i]           = local_prim++;
    }

    for(auto&& [i, local_prim] : enumerate(region_prim_count))
    {
        L2G[i].resize(local_prim);
    }

    for(auto&& [i, local_prim] : enumerate(G2L))
    {
        auto r             = region[i];
        L2G[r][local_prim] = i;
    }
}

vector<SimplicialComplex> apply_region(const SimplicialComplex& sc)
{
    auto region_count = sc.meta().find<IndexT>("region_count");

    UIPC_ASSERT(region_count,
                "The `region_count` attribute is not found in the complex. "
                "You need to call label_region() to label the region of the geometry");

    auto   region_count_view = region_count->view();
    IndexT N_region          = region_count_view[0];
    IndexT Dim               = sc.dim();

    if(Dim >= 1)
    {
        auto edge_region = sc.edges().find<IndexT>("region");
        UIPC_ASSERT(edge_region,
                    "The `region` attribute is not found in the edges. "
                    "You need to call label_region() to label the region of the geometry");
    }

    vector<SimplicialComplex> Rs(N_region);

    // 1) calculate the vertex mapping
    vector<SizeT>         global_vert_to_local_vert;
    vector<vector<SizeT>> local_vert_to_global_vert;
    auto                  vert_region = sc.vertices().find<IndexT>("region");
    auto                  vert_region_view = vert_region->view();
    calculate_mapping(N_region, vert_region_view, global_vert_to_local_vert, local_vert_to_global_vert);


    auto GV2LV = [&global_vert_to_local_vert]<int N>(const Eigen::Vector<IndexT, N>& GV)  // Map the global vertex to local vertex
    {
        Eigen::Vector<IndexT, N> ret;
        for(int i = 0; i < N; i++)
        {
            ret[i] = static_cast<IndexT>(global_vert_to_local_vert[GV[i]]);
        }
        return ret;
    };

    // 2) copy the meta
    {
        vector<std::string> excluding_attributes{"region_count"};  // exclude the region_count attribute

        for(auto&& [region_I, R] : enumerate(Rs))
        {
            R.meta().copy_from(sc.meta(), {}, {}, excluding_attributes);
        }
    }

    // 3) copy the instances
    {
        for(auto&& [region_I, R] : enumerate(Rs))
        {
            R.instances().copy_from(sc.instances());
        }
    }

    // 4) copy the vertices
    {
        vector<std::string> excluding_attributes{"region"};  // exclude the region attribute

        for(auto&& [region_I, R] : enumerate(Rs))
        {
            auto& L2G = local_vert_to_global_vert[region_I];

            R.vertices().resize(L2G.size());

            R.vertices().copy_from(sc.vertices(),
                                   AttributeCopy::pull(L2G),  // pull the vertices
                                   {},  // default all attributes
                                   excluding_attributes  // exclude
            );
        }
    }

    // 5) copy the edges
    if(Dim >= 1)
    {
        vector<SizeT>         global_edge_to_local_edge;
        vector<vector<SizeT>> local_edge_to_global_edge;

        auto edge_region      = sc.edges().find<IndexT>("region");
        auto edge_region_view = edge_region->view();
        calculate_mapping(N_region, edge_region_view, global_edge_to_local_edge, local_edge_to_global_edge);


        // exclude:
        // - topo, we need to fill it by ourselves
        // - region, we don't need to copy it
        vector<std::string> excluding_attributes{"topo", "region"};

        auto src_edge_topo_view = sc.edges().topo().view();

        for(auto&& [region_I, R] : enumerate(Rs))
        {
            auto& L2G = local_edge_to_global_edge[region_I];

            auto edge_topo = R.edges().create<Vector2i>("topo");
            R.edges().resize(L2G.size());

            auto edge_topo_view = view(*edge_topo);

            // setup topo
            for(auto&& [local_I, global_I] : enumerate(L2G))
            {
                // src edge = (GV1, GV2)
                auto src_edge = src_edge_topo_view[global_I];

                // dst edge = (LV1, LV2)
                Vector2i dst_edge = GV2LV(src_edge);

                edge_topo_view[local_I] = dst_edge;
            }

            // copy the attributes
            R.edges().copy_from(sc.edges(),
                                AttributeCopy::pull(L2G),  // pull the edges
                                {},                   // default all attributes
                                excluding_attributes  // exclude
            );
        }
    }

    // 6) copy the triangles
    if(Dim >= 2)
    {
        vector<SizeT>         global_tri_to_local_tri;
        vector<vector<SizeT>> local_tri_to_global_tri;

        auto tri_region      = sc.triangles().find<IndexT>("region");
        auto tri_region_view = tri_region->view();
        calculate_mapping(N_region, tri_region_view, global_tri_to_local_tri, local_tri_to_global_tri);

        // exclude:
        // - topo, we need to fill it by ourselves
        // - region, we don't need to copy it
        vector<std::string> excluding_attributes{"topo", "region"};

        auto src_tri_topo_view = sc.triangles().topo().view();

        for(auto&& [region_I, R] : enumerate(Rs))
        {
            auto& L2G = local_tri_to_global_tri[region_I];

            auto tri_topo = R.triangles().create<Vector3i>("topo");
            R.triangles().resize(L2G.size());

            auto tri_topo_view = view(*tri_topo);

            // setup topo
            for(auto&& [local_I, global_I] : enumerate(L2G))
            {
                // src tri = (GV1, GV2, GV3)
                auto src_tri = src_tri_topo_view[global_I];

                // dst tri = (LV1, LV2, LV3)
                Vector3i dst_tri = GV2LV(src_tri);

                tri_topo_view[local_I] = dst_tri;
            }

            // copy the attributes
            R.triangles().copy_from(sc.triangles(),
                                    AttributeCopy::pull(L2G),  // pull the triangles
                                    {},  // default all attributes
                                    excluding_attributes  // exclude
            );
        }
    }

    // 7) copy the tetrahedra
    if(Dim >= 3)
    {
        vector<SizeT>         global_tet_to_local_tet;
        vector<vector<SizeT>> local_tet_to_global_tet;

        auto tet_region      = sc.tetrahedra().find<IndexT>("region");
        auto tet_region_view = tet_region->view();
        calculate_mapping(N_region, tet_region_view, global_tet_to_local_tet, local_tet_to_global_tet);

        // exclude:
        // - topo, we need to fill it by ourselves
        // - region, we don't need to copy it
        vector<std::string> excluding_attributes{"topo", "region"};

        auto src_tet_topo_view = sc.tetrahedra().topo().view();

        for(auto&& [region_I, R] : enumerate(Rs))
        {
            auto& L2G = local_tet_to_global_tet[region_I];

            auto tet_topo = R.tetrahedra().create<Vector4i>("topo");
            R.tetrahedra().resize(L2G.size());

            auto tet_topo_view = view(*tet_topo);

            // setup topo
            for(auto&& [local_I, global_I] : enumerate(L2G))
            {
                // src tet = (GV1, GV2, GV3, GV4)
                auto src_tet = src_tet_topo_view[global_I];

                // dst tet = (LV1, LV2, LV3, LV4)
                Vector4i dst_tet = GV2LV(src_tet);

                tet_topo_view[local_I] = dst_tet;
            }

            // copy the attributes
            R.tetrahedra().copy_from(sc.tetrahedra(),
                                     AttributeCopy::pull(L2G),  // pull the tetrahedra
                                     {},  // default all attributes
                                     excluding_attributes  // exclude
            );
        }
    }

    return Rs;
}
}  // namespace uipc::geometry
