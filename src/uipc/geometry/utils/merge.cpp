#include <uipc/geometry/utils/merge.h>
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

namespace detail
{
    template <bool OnlyMergeTopDim>
    static SimplicialComplex merge(span<const SimplicialComplex*> complexes)
    {
        check_merge_input(complexes);

        SimplicialComplex R;

        vector<SizeT> vertex_offsets(complexes.size());
        vector<SizeT> vertex_counts(complexes.size());

        std::ranges::transform(complexes,
                               vertex_counts.begin(),
                               [](const SimplicialComplex* complex)
                               { return complex->vertices().size(); });

        std::exclusive_scan(
            vertex_counts.begin(), vertex_counts.end(), vertex_offsets.begin(), 0);


        {  // 1) merge vertices
            SizeT num_verts = vertex_offsets.back() + vertex_counts.back();

            R.vertices().resize(num_verts);

            for(SizeT dst_offset = 0; auto complex : complexes)
            {
                auto count = complex->vertices().size();
                R.vertices().copy_from(complex->vertices(),
                                       AttributeCopy::range(dst_offset, 0, count));
                dst_offset += count;
            }
        }

        {  // 2) merge edges

            vector<SizeT> edge_offsets(complexes.size());
            vector<SizeT> edge_counts(complexes.size());

            std::ranges::transform(complexes,
                                   edge_counts.begin(),
                                   [](const SimplicialComplex* complex)
                                   {
                                       if constexpr(OnlyMergeTopDim)
                                       {
                                           return complex->dim() == 1 ?
                                                      complex->edges().size() :
                                                      0;
                                       }
                                       else
                                       {
                                           return complex->edges().size();
                                       }
                                   });

            std::exclusive_scan(
                edge_counts.begin(), edge_counts.end(), edge_offsets.begin(), 0);

            SizeT num_edges = edge_offsets.back() + edge_counts.back();

            R.edges().resize(num_edges);

            auto Es = view(R.edges().topo());


            // setup edge topology
            for(auto [I, complex] : enumerate(complexes))
            {
                SizeT e_offset = edge_offsets[I];
                SizeT e_count  = edge_counts[I];
                SizeT v_offset = vertex_offsets[I];

                span<Vector2i>       dst_Es = Es.subspan(e_offset, e_count);
                span<const Vector2i> src_Es = complex->edges().topo().view();

                // add offset to each edge
                std::ranges::transform(src_Es,
                                       dst_Es.begin(),
                                       [=](const Vector2i& e) -> Vector2i
                                       { return e.array() + v_offset; });
            }

            // copy edge attributes
            for(SizeT dst_offset = 0; auto complex : complexes)
            {
                auto count = complex->edges().size();
                if(count > 0)
                {
                    R.edges().copy_from(complex->edges(),
                                        AttributeCopy::range(dst_offset, 0, count));
                }
                dst_offset += count;
            }
        }

        {  // 3) merge triangles

            vector<SizeT> triangle_offsets(complexes.size());
            vector<SizeT> triangle_counts(complexes.size());

            std::ranges::transform(complexes,
                                   triangle_counts.begin(),
                                   [](const SimplicialComplex* complex)
                                   {
                                       if constexpr(OnlyMergeTopDim)
                                       {
                                           return complex->dim() == 2 ?
                                                      complex->triangles().size() :
                                                      0;
                                       }
                                       else
                                       {
                                           return complex->triangles().size();
                                       }
                                   });
            std::exclusive_scan(triangle_counts.begin(),
                                triangle_counts.end(),
                                triangle_offsets.begin(),
                                0);

            SizeT num_triangles = triangle_offsets.back() + triangle_counts.back();

            R.triangles().resize(num_triangles);

            auto Fs = view(R.triangles().topo());

            // setup triangle topology
            for(auto [I, complex] : enumerate(complexes))
            {
                SizeT t_offset = triangle_offsets[I];
                SizeT t_count  = triangle_counts[I];
                SizeT v_offset = vertex_offsets[I];

                span<Vector3i> dst_Ts = Fs.subspan(t_offset, t_count);
                span<const Vector3i> src_Ts = complex->triangles().topo().view();

                // add offset to each triangle
                std::ranges::transform(src_Ts,
                                       dst_Ts.begin(),
                                       [=](const Vector3i& t) -> Vector3i
                                       { return t.array() + v_offset; });
            }

            // copy triangle attributes
            for(SizeT dst_offset = 0; auto complex : complexes)
            {
                auto count = complex->triangles().size();
                if(count > 0)
                {
                    R.triangles().copy_from(complex->triangles(),
                                            AttributeCopy::range(dst_offset, 0, count));
                }
                dst_offset += count;
            }
        }

        {  // 4) merge tetrahedra

            vector<SizeT> tetrahedron_offsets(complexes.size());
            vector<SizeT> tetrahedron_counts(complexes.size());

            std::ranges::transform(complexes,
                                   tetrahedron_counts.begin(),
                                   [](const SimplicialComplex* complex)
                                   {
                                       if constexpr(OnlyMergeTopDim)
                                       {
                                           return complex->dim() == 3 ?
                                                      complex->tetrahedra().size() :
                                                      0;
                                       }
                                       else
                                       {
                                           return complex->tetrahedra().size();
                                       }
                                   });

            std::exclusive_scan(tetrahedron_counts.begin(),
                                tetrahedron_counts.end(),
                                tetrahedron_offsets.begin(),
                                0);

            SizeT num_tetrahedra =
                tetrahedron_offsets.back() + tetrahedron_counts.back();

            R.tetrahedra().resize(num_tetrahedra);

            auto Ts = view(R.tetrahedra().topo());

            // setup tetrahedron topology
            for(auto [I, complex] : enumerate(complexes))
            {
                SizeT t_offset = tetrahedron_offsets[I];
                SizeT t_count  = tetrahedron_counts[I];
                SizeT v_offset = vertex_offsets[I];

                span<Vector4i> dst_Ts = Ts.subspan(t_offset, t_count);
                span<const Vector4i> src_Ts = complex->tetrahedra().topo().view();

                // add offset to each tetrahedron
                std::ranges::transform(src_Ts,
                                       dst_Ts.begin(),
                                       [=](const Vector4i& t) -> Vector4i
                                       { return t.array() + v_offset; });
            }

            // copy tetrahedron attributes
            for(SizeT dst_offset = 0; auto complex : complexes)
            {
                auto count = complex->tetrahedra().size();
                if(count > 0)
                {
                    R.tetrahedra().copy_from(complex->tetrahedra(),
                                             AttributeCopy::range(dst_offset, 0, count));
                }
                dst_offset += count;
            }
        }

        return R;
    }
}  // namespace detail

SimplicialComplex merge(span<const SimplicialComplex*> complexes)
{
    return detail::merge<false>(complexes);
}

SimplicialComplex merge_top_dim(span<const SimplicialComplex*> complexes)
{
    return detail::merge<true>(complexes);
}
}  // namespace uipc::geometry
