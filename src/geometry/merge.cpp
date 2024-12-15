#include <uipc/geometry/utils/merge.h>
#include <uipc/builtin/attribute_name.h>
#include <numeric>
#include <algorithm>
#include <ranges>

namespace uipc::geometry
{
static void check_merge_input(span<const SimplicialComplex*> complexes)
{
    for(auto [I, complex] : enumerate(complexes))
    {
        UIPC_ASSERT(complex != nullptr, "complex[{}] is nullptr", I);
        UIPC_ASSERT(complex->instances().size() == 1,
                    "complex[{}] has multiple instances ({}), call `apply_instances()` before merging multiple-instance-geometry",
                    I,
                    complex->instances().size());
    }
}


SimplicialComplex merge(span<const SimplicialComplex*> complexes)
{
    check_merge_input(complexes);

    SimplicialComplex R;

    vector<SizeT> vertex_offsets(complexes.size() + 1, 0);
    vector<SizeT> vertex_counts(complexes.size() + 1, 0);

    std::ranges::transform(complexes,
                           vertex_counts.begin(),
                           [](const SimplicialComplex* complex)
                           { return complex->vertices().size(); });

    std::exclusive_scan(
        vertex_counts.begin(), vertex_counts.end(), vertex_offsets.begin(), 0ull);

    vector<string> exclude_attributes = {string{builtin::topo}};

    {  // 1) merge vertices
        SizeT num_verts = vertex_offsets.back();

        R.vertices().resize(num_verts);

        for(auto&& [i, complex] : enumerate(complexes))
        {
            auto count  = vertex_counts[i];
            auto offset = vertex_offsets[i];
            R.vertices().copy_from(complex->vertices(),
                                   AttributeCopy::range(offset, 0, count),
                                   {},
                                   exclude_attributes);
        }
    }

    {  // 2) merge edges

        vector<SizeT> edge_offsets(complexes.size() + 1, 0);
        vector<SizeT> edge_counts(complexes.size() + 1, 0);

        std::ranges::transform(complexes,
                               edge_counts.begin(),
                               [](const SimplicialComplex* complex)
                               { return complex->edges().size(); });

        std::exclusive_scan(edge_counts.begin(), edge_counts.end(), edge_offsets.begin(), 0ull);

        SizeT num_edges = edge_offsets.back();

        R.edges().resize(num_edges);
        auto topo = R.edges().create<Vector2i>(builtin::topo, Vector2i::Zero(), false);
        auto Es = view(*topo);


        // setup edge topology
        for(auto&& [I, complex] : enumerate(complexes))
        {
            SizeT e_offset = edge_offsets[I];
            SizeT e_count  = edge_counts[I];
            SizeT v_offset = vertex_offsets[I];

            if(e_count)
            {
                span<Vector2i>       dst_Es = Es.subspan(e_offset, e_count);
                span<const Vector2i> src_Es = complex->edges().topo().view();

                // add offset to each edge
                std::ranges::transform(src_Es,
                                       dst_Es.begin(),
                                       [=](const Vector2i& e) -> Vector2i
                                       { return e.array() + v_offset; });
            }
        }

        // copy edge attributes
        for(auto&& [I, complex] : enumerate(complexes))
        {
            auto count  = edge_counts[I];
            auto offset = edge_offsets[I];
            if(count > 0)
            {
                R.edges().copy_from(complex->edges(),
                                    AttributeCopy::range(offset, 0, count),
                                    {},
                                    exclude_attributes);
            }
        }
    }

    {  // 3) merge triangles

        vector<SizeT> triangle_offsets(complexes.size() + 1, 0);
        vector<SizeT> triangle_counts(complexes.size() + 1, 0);

        std::ranges::transform(complexes,
                               triangle_counts.begin(),
                               [](const SimplicialComplex* complex)
                               { return complex->triangles().size(); });

        std::exclusive_scan(triangle_counts.begin(),
                            triangle_counts.end(),
                            triangle_offsets.begin(),
                            0ull);

        SizeT num_triangles = triangle_offsets.back();

        R.triangles().resize(num_triangles);
        auto topo = R.triangles().create<Vector3i>(builtin::topo, Vector3i::Zero(), false);

        auto Fs = view(*topo);

        // setup triangle topology
        for(auto [I, complex] : enumerate(complexes))
        {
            SizeT offset   = triangle_offsets[I];
            SizeT count    = triangle_counts[I];
            SizeT v_offset = vertex_offsets[I];

            if(count)
            {
                span<Vector3i> dst_Ts = Fs.subspan(offset, count);
                span<const Vector3i> src_Ts = complex->triangles().topo().view();

                // add offset to each triangle
                std::ranges::transform(src_Ts,
                                       dst_Ts.begin(),
                                       [=](const Vector3i& t) -> Vector3i
                                       { return t.array() + v_offset; });
            }
        }

        // copy triangle attributes
        for(auto [I, complex] : enumerate(complexes))
        {
            auto count = triangle_counts[I];
            if(count > 0)
            {
                R.triangles().copy_from(complex->triangles(),
                                        AttributeCopy::range(triangle_offsets[I], 0, count),
                                        {},
                                        exclude_attributes);
            }
        }
    }

    {  // 4) merge tetrahedra

        vector<SizeT> tetrahedron_offsets(complexes.size() + 1, 0);
        vector<SizeT> tetrahedron_counts(complexes.size() + 1, 0);

        std::ranges::transform(complexes,
                               tetrahedron_counts.begin(),
                               [](const SimplicialComplex* complex)
                               { return complex->tetrahedra().size(); });

        std::exclusive_scan(tetrahedron_counts.begin(),
                            tetrahedron_counts.end(),
                            tetrahedron_offsets.begin(),
                            0ull);

        SizeT num_tetrahedra = tetrahedron_offsets.back();

        R.tetrahedra().resize(num_tetrahedra);
        auto topo = R.tetrahedra().create<Vector4i>(builtin::topo, Vector4i::Zero(), false);
        auto Ts = view(*topo);

        // setup tetrahedron topology
        for(auto [I, complex] : enumerate(complexes))
        {
            SizeT t_offset = tetrahedron_offsets[I];
            SizeT t_count  = tetrahedron_counts[I];
            SizeT v_offset = vertex_offsets[I];

            if(t_count)
            {
                span<Vector4i> dst_Ts = Ts.subspan(t_offset, t_count);
                span<const Vector4i> src_Ts = complex->tetrahedra().topo().view();

                // add offset to each tetrahedron
                std::ranges::transform(src_Ts,
                                       dst_Ts.begin(),
                                       [=](const Vector4i& t) -> Vector4i
                                       { return t.array() + v_offset; });
            }
        }

        // copy tetrahedron attributes
        for(auto [I, complex] : enumerate(complexes))
        {
            auto count = tetrahedron_counts[I];
            if(count > 0)
            {
                R.tetrahedra().copy_from(complex->tetrahedra(),
                                         AttributeCopy::range(tetrahedron_offsets[I], 0, count),
                                         {},
                                         exclude_attributes);
            }
        }
    }

    return R;
}

SimplicialComplex merge(std::initializer_list<const SimplicialComplex*>&& complexes)
{
    vector<const SimplicialComplex*> complex_ptrs{complexes.begin(), complexes.end()};
    return merge(complex_ptrs);
}
}  // namespace uipc::geometry
