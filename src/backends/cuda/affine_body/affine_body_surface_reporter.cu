#include <affine_body/affine_body_surface_reporter.h>
#include <uipc/builtin/attribute_name.h>
#include <global_geometry/global_vertex_manager.h>
#include <uipc/common/range.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffinebodySurfaceReporter);

void AffinebodySurfaceReporter::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics        = &require<AffineBodyDynamics>();
    auto& global_vertex_manager        = require<GlobalVertexManager>();
    m_impl.affine_body_vertex_reporter = &require<AffineBodyVertexReporter>();
}

void AffinebodySurfaceReporter::Impl::init(backend::WorldVisitor& world)
{
    auto abd_vertex_offset_in_global = affine_body_vertex_reporter->vertex_offset();

    auto abd_geo_count = abd().abd_geo_count;

    // +1 for total count
    vector<SizeT> geo_surf_vertex_counts(abd_geo_count + 1, 0);
    vector<SizeT> geo_surf_vertex_offsets(abd_geo_count + 1, 0);

    vector<SizeT> geo_surf_edges_counts(abd_geo_count + 1, 0);
    vector<SizeT> geo_surf_edges_offsets(abd_geo_count + 1, 0);

    vector<SizeT> geo_surf_triangles_counts(abd_geo_count + 1, 0);
    vector<SizeT> geo_surf_triangles_offsets(abd_geo_count + 1, 0);

    auto geo_slots = world.scene().geometries();

    // 1) count surf vertices, edges, triangles in each geometry
    {
        SizeT geoI = 0;
        abd().for_each(  //
            geo_slots,
            [&](geometry::SimplicialComplex& sc)
            {
                auto body_count = sc.instances().size();

                {  // vertex
                    auto is_surf = sc.vertices().find<IndexT>(builtin::is_surf);

                    auto is_surf_view = is_surf ? is_surf->view() : span<const IndexT>{};

                    auto count = std::ranges::count_if(is_surf_view,
                                                       [](const IndexT& is_surf)
                                                       { return is_surf; });
                    geo_surf_vertex_counts[geoI] = count * body_count;
                }

                {  // edge
                    auto is_surf = sc.edges().find<IndexT>(builtin::is_surf);
                    auto is_surf_view = is_surf ? is_surf->view() : span<const IndexT>{};
                    auto count = std::ranges::count_if(is_surf_view,
                                                       [](const IndexT& is_surf)
                                                       { return is_surf; });
                    geo_surf_edges_counts[geoI] = count * body_count;
                }

                {  // triangle
                    auto is_surf = sc.triangles().find<IndexT>(builtin::is_surf);
                    auto is_surf_view = is_surf ? is_surf->view() : span<const IndexT>{};
                    auto count = std::ranges::count_if(is_surf_view,
                                                       [](const IndexT& is_surf)
                                                       { return is_surf; });
                    geo_surf_triangles_counts[geoI] = count * body_count;
                }

                geoI++;
            });
    }

    // 2) calculate offsets
    std::exclusive_scan(geo_surf_vertex_counts.begin(),
                        geo_surf_vertex_counts.end(),
                        geo_surf_vertex_offsets.begin(),
                        0);
    std::exclusive_scan(geo_surf_edges_counts.begin(),
                        geo_surf_edges_counts.end(),
                        geo_surf_edges_offsets.begin(),
                        0);
    std::exclusive_scan(geo_surf_triangles_counts.begin(),
                        geo_surf_triangles_counts.end(),
                        geo_surf_triangles_offsets.begin(),
                        0);

    auto total_surf_vertex_count   = geo_surf_vertex_offsets.back();
    auto total_surf_edge_count     = geo_surf_edges_offsets.back();
    auto total_surf_triangle_count = geo_surf_triangles_offsets.back();

    surf_vertices.resize(total_surf_vertex_count, -1);
    surf_edges.resize(total_surf_edge_count, -Vector2i::Ones());
    surf_triangles.resize(total_surf_triangle_count, -Vector3i::Ones());

    // 3) fill surf vertices, edges, triangles
    {
        vector<IndexT> surf_vertex_cache;
        vector<IndexT> surf_edge_cache;
        vector<IndexT> surf_triangle_cache;

        SizeT geoI = 0;
        abd().for_each(
            geo_slots,
            [&](geometry::SimplicialComplex& sc)
            {
                auto& geo_info   = abd().geo_infos[geoI];
                auto  body_count = sc.instances().size();

                auto geo_vertex_offset_in_global = geo_info.vertex_offset  // offset in affine body vertex
                                                   + abd_vertex_offset_in_global;


                {  // surf vertex
                    auto geo_surf_vertex_count = geo_surf_vertex_counts[geoI];

                    UIPC_ASSERT(geo_surf_vertex_count % body_count == 0,
                                "surf vertex count is not multiple of body count, why can it happen?");


                    // only need to cache the first body
                    auto body_surf_vertex_count = geo_surf_vertex_count / body_count;

                    surf_vertex_cache.clear();
                    surf_vertex_cache.reserve(body_surf_vertex_count);
                    auto is_surf = sc.vertices().find<IndexT>(builtin::is_surf);
                    auto is_surf_view = is_surf ? is_surf->view() : span<const IndexT>{};

                    for(auto&& [i, is_surf] : enumerate(is_surf_view))
                    {
                        if(is_surf)
                            surf_vertex_cache.push_back(i);
                    }

                    UIPC_ASSERT(surf_vertex_cache.size() == body_surf_vertex_count,
                                "surf vertex cache size is not equal to body_surf_vertex_count, why can it happen?");

                    auto body_vertex_count = geo_info.vertex_count / body_count;

                    for(auto i : range(body_count))
                    {
                        auto body_vertex_offset_in_global =
                            geo_vertex_offset_in_global + i * body_vertex_count;

                        auto surf_v = span{surf_vertices}.subspan(
                            geo_surf_vertex_offsets[geoI]
                                + i * surf_vertex_cache.size(),
                            surf_vertex_cache.size());

                        std::ranges::transform(surf_vertex_cache,
                                               surf_v.begin(),
                                               [&](const IndexT& local_surf_vert_id) {
                                                   return local_surf_vert_id + body_vertex_offset_in_global;
                                               });

                        for(auto&& [i, v] : enumerate(surf_v))
                        {
                            if(v == 0)
                            {
                                fmt::println("surf vertex {}: {}", i, v);
                            }
                        }
                    }
                }

                {  // surf edge
                    auto geo_surf_edge_count = geo_surf_edges_counts[geoI];

                    UIPC_ASSERT(geo_surf_edge_count % body_count == 0,
                                "surf edge count is not multiple of body count, why can it happen?");


                    // only need to cache the first body
                    auto body_surf_edge_count = geo_surf_edge_count / body_count;

                    surf_edge_cache.clear();
                    surf_edge_cache.reserve(body_surf_edge_count);

                    auto is_surf = sc.edges().find<IndexT>(builtin::is_surf);
                    auto is_surf_view = is_surf ? is_surf->view() : span<const IndexT>{};

                    for(auto&& [i, is_surf] : enumerate(is_surf_view))
                    {
                        if(is_surf)
                            surf_edge_cache.push_back(i);
                    }

                    UIPC_ASSERT(surf_edge_cache.size() == body_surf_edge_count,
                                "surf edge cache size is not equal to body_surf_edge_count, why can it happen?");

                    auto body_vertex_count = geo_info.vertex_count / body_count;

                    for(auto i : range(body_count))
                    {
                        auto body_vertex_offset_in_global =
                            geo_vertex_offset_in_global + i * body_vertex_count;

                        auto surf_e = span{surf_edges}.subspan(
                            geo_surf_edges_offsets[geoI] + i * surf_edge_cache.size(),
                            surf_edge_cache.size());

                        auto Es = sc.edges().topo().view();

                        std::ranges::transform(surf_edge_cache,
                                               surf_e.begin(),
                                               [&](const IndexT& local_surf_edge_id) -> Vector2i
                                               {
                                                   auto edge = Es[local_surf_edge_id];
                                                   Vector2i ret = edge.array() + body_vertex_offset_in_global;
                                                   return ret;
                                               });
                    }
                }

                {  // surf triangle

                    auto geo_surf_triangle_count = geo_surf_triangles_counts[geoI];

                    UIPC_ASSERT(geo_surf_triangle_count % body_count == 0,
                                "surf triangle count is not multiple of body count, why can it happen?");

                    // only need to cache the first body
                    auto body_surf_triangle_count = geo_surf_triangle_count / body_count;

                    surf_triangle_cache.clear();
                    surf_triangle_cache.reserve(body_surf_triangle_count);

                    auto is_surf = sc.triangles().find<IndexT>(builtin::is_surf);
                    auto is_surf_view = is_surf ? is_surf->view() : span<const IndexT>{};

                    for(auto&& [i, is_surf] : enumerate(is_surf_view))
                    {
                        if(is_surf)
                            surf_triangle_cache.push_back(i);
                    }

                    UIPC_ASSERT(surf_triangle_cache.size() == body_surf_triangle_count,
                                "surf triangle cache size is not equal to body_surf_triangle_count, why can it happen?");

                    auto body_vertex_count = geo_info.vertex_count / body_count;

                    for(auto i : range(body_count))
                    {
                        auto body_vertex_offset_in_global =
                            geo_vertex_offset_in_global + i * body_vertex_count;

                        auto surf_f = span{surf_triangles}.subspan(
                            geo_surf_triangles_offsets[geoI]
                                + i * surf_triangle_cache.size(),
                            surf_triangle_cache.size());

                        auto Fs = sc.triangles().topo().view();

                        std::ranges::transform(surf_triangle_cache,
                                               surf_f.begin(),
                                               [&](const IndexT& local_surf_tri_id) -> Vector3i
                                               {
                                                   auto tri = Fs[local_surf_tri_id];
                                                   Vector3i ret = tri.array() + body_vertex_offset_in_global;
                                                   return ret;
                                               });
                    }
                }

                geoI++;
            });
    }
}

void AffinebodySurfaceReporter::Impl::report_count(backend::WorldVisitor& world,
                                                   SurfaceCountInfo&      info)
{
    info.surf_vertex_count(surf_vertices.size());
    info.surf_edge_count(surf_edges.size());
    info.surf_triangle_count(surf_triangles.size());
}

void AffinebodySurfaceReporter::Impl::report_attributes(backend::WorldVisitor& world,
                                                        SurfaceAttributeInfo& info)
{
    auto async_copy = []<typename T>(span<T> src, muda::BufferView<T> dst)
    { muda::BufferLaunch().copy<T>(dst, src.data()); };

    async_copy(span{surf_vertices}, info.surf_vertices());
    async_copy(span{surf_edges}, info.surf_edges());
    async_copy(span{surf_triangles}, info.surf_triangles());
}

void AffinebodySurfaceReporter::do_init(SurfaceInitInfo& info)
{
    m_impl.init(world());
}

void AffinebodySurfaceReporter::do_report_count(SurfaceCountInfo& info)
{
    m_impl.report_count(world(), info);
}

void AffinebodySurfaceReporter::do_report_attributes(SurfaceAttributeInfo& info)
{
    m_impl.report_attributes(world(), info);
}
}  // namespace uipc::backend::cuda
