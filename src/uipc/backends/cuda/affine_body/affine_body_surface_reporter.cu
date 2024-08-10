#include <affine_body/affine_body_surface_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffinebodySurfaceReporter);

void AffinebodySurfaceReporter::do_build()
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
    auto& global_surf_manager   = require<GlobalSimpicialSurfaceManager>();
    m_impl.affine_body_vertex_reporter = &require<AffineBodyVertexReporter>();

    m_impl.affine_body_dynamics->after_build_geometry(
        *this, [this] { m_impl.init_surface(world()); });
    global_surf_manager.add_reporter(this);
}

void AffinebodySurfaceReporter::Impl::init_surface(backend::WorldVisitor& world)
{
    _init_geo_surface(world);
    _init_body_surface(world);
}

void AffinebodySurfaceReporter::Impl::_init_geo_surface(backend::WorldVisitor& world)
{
    auto geo_slots = world.scene().geometries();

    auto geo_count = abd().abd_geo_count;

    auto N = geo_count + 1;  // +1 for total count

    geo_surf_vertex_offsets.resize(N);
    geo_surf_edges_offsets.resize(N);
    geo_surf_triangles_offsets.resize(N);

    geo_surf_vertex_counts.resize(N);
    geo_surf_edges_counts.resize(N);
    geo_surf_triangles_counts.resize(N);

    geo_surf_vertex_counts.back()    = 0;
    geo_surf_edges_counts.back()     = 0;
    geo_surf_triangles_counts.back() = 0;

    // 1) for every geometry, count surface primitives
    for(auto&& [i, body_offset] : enumerate(abd().h_abd_geo_body_offsets))
    {
        const auto& body_info = abd().h_body_infos[body_offset];
        auto&       geo       = abd().geometry(geo_slots, body_info);

        auto V_is_surf = geo.vertices().find<IndexT>(builtin::is_surf);
        if(!V_is_surf)
        {
            geo_surf_vertex_counts[i] = 0;
        }
        else
        {
            auto is_surf_view         = V_is_surf->view();
            geo_surf_vertex_counts[i] = std::ranges::count_if(
                is_surf_view, [](const IndexT& is_surf) { return is_surf; });
        }

        auto E_is_surf = geo.edges().find<IndexT>(builtin::is_surf);
        if(!E_is_surf)
        {
            geo_surf_edges_counts[i] = 0;
        }
        else
        {
            auto is_surf_view        = E_is_surf->view();
            geo_surf_edges_counts[i] = std::ranges::count_if(
                is_surf_view, [](const IndexT& is_surf) { return is_surf; });
        }

        auto F_is_surf = geo.triangles().find<IndexT>(builtin::is_surf);
        if(!F_is_surf)
        {
            geo_surf_triangles_counts[i] = 0;
        }
        else
        {
            auto is_surf_view            = F_is_surf->view();
            geo_surf_triangles_counts[i] = std::ranges::count_if(
                is_surf_view, [](const IndexT& is_surf) { return is_surf; });
        }
    }

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

    total_geo_surf_vertex_count   = geo_surf_vertex_offsets.back();
    total_geo_surf_edge_count     = geo_surf_edges_offsets.back();
    total_geo_surf_triangle_count = geo_surf_triangles_offsets.back();

    geo_local_surf_vertices.resize(total_geo_surf_vertex_count);
    geo_local_surf_edges.resize(total_geo_surf_edge_count);
    geo_local_surf_triangles.resize(total_geo_surf_triangle_count);

    // 2) for every geometry, collect surface primitive id
    auto collect_surf_id = [](span<const IndexT> is_surf, span<IndexT> surf_id)
    {
        SizeT idx = 0;
        for(auto&& [i, is_surf] : enumerate(is_surf))
        {
            if(is_surf)
            {
                surf_id[idx++] = i;
            }
        }
    };

    for(auto&& [i, body_offset] : enumerate(abd().h_abd_geo_body_offsets))
    {
        const auto& body_info = abd().h_body_infos[body_offset];
        auto&       geo       = abd().geometry(geo_slots, body_info);

        if(geo_surf_vertex_counts[i] > 0)
        {
            auto surf_v =
                span{geo_local_surf_vertices}.subspan(geo_surf_vertex_offsets[i],
                                                      geo_surf_vertex_counts[i]);
            auto is_surf = geo.vertices().find<IndexT>(builtin::is_surf)->view();
            collect_surf_id(is_surf, surf_v);
        }

        if(geo_surf_edges_counts[i] > 0)
        {
            auto surf_e =
                span{geo_local_surf_edges}.subspan(geo_surf_edges_offsets[i],
                                                   geo_surf_edges_counts[i]);
            auto is_surf = geo.edges().find<IndexT>(builtin::is_surf)->view();
            collect_surf_id(is_surf, surf_e);
        }

        if(geo_surf_triangles_counts[i] > 0)
        {
            auto surf_f = span{geo_local_surf_triangles}.subspan(
                geo_surf_triangles_offsets[i], geo_surf_triangles_counts[i]);
            auto is_surf = geo.triangles().find<IndexT>(builtin::is_surf)->view();
            collect_surf_id(is_surf, surf_f);
        }
    }
}

void AffinebodySurfaceReporter::Impl::_init_body_surface(backend::WorldVisitor& world)
{
    auto geo_slots = world.scene().geometries();

    body_surface_infos.resize(abd().abd_body_count);

    auto           N = abd().abd_body_count + 1;  // +1 for total count
    vector<IndexT> body_surf_vertex_offsets(N);
    vector<IndexT> body_surf_edges_offsets(N);
    vector<IndexT> body_surf_triangles_offsets(N);

    body_surf_vertex_offsets.back()    = 0;
    body_surf_edges_offsets.back()     = 0;
    body_surf_triangles_offsets.back() = 0;

    // 1) for every body, count surface primitives
    for(auto&& [body_info, body_surf_info] : zip(abd().h_body_infos, body_surface_infos))
    {
        auto geo_index  = body_info.abd_geometry_index();
        auto body_index = body_info.affine_body_id();

        body_surf_info.m_surf_vertex_count = geo_surf_vertex_counts[geo_index];
        body_surf_vertex_offsets[body_index] = body_surf_info.m_surf_vertex_count;

        body_surf_info.m_surf_edge_count    = geo_surf_edges_counts[geo_index];
        body_surf_edges_offsets[body_index] = body_surf_info.m_surf_edge_count;

        body_surf_info.m_surf_triangle_count = geo_surf_triangles_counts[geo_index];
        body_surf_triangles_offsets[body_index] = body_surf_info.m_surf_triangle_count;
    }

    std::exclusive_scan(body_surf_vertex_offsets.begin(),
                        body_surf_vertex_offsets.end(),
                        body_surf_vertex_offsets.begin(),
                        0);
    std::exclusive_scan(body_surf_edges_offsets.begin(),
                        body_surf_edges_offsets.end(),
                        body_surf_edges_offsets.begin(),
                        0);
    std::exclusive_scan(body_surf_triangles_offsets.begin(),
                        body_surf_triangles_offsets.end(),
                        body_surf_triangles_offsets.begin(),
                        0);

    for(auto&& [i, body_surf_info] : enumerate(body_surface_infos))
    {
        body_surf_info.m_surf_vertex_offset   = body_surf_vertex_offsets[i];
        body_surf_info.m_surf_edge_offset     = body_surf_edges_offsets[i];
        body_surf_info.m_surf_triangle_offset = body_surf_triangles_offsets[i];
    }

    total_surf_vertex_count   = body_surf_vertex_offsets.back();
    total_surf_edge_count     = body_surf_edges_offsets.back();
    total_surf_triangle_count = body_surf_triangles_offsets.back();

    surf_vertices.resize(total_surf_vertex_count);
    surf_edges.resize(total_surf_edge_count);
    surf_triangles.resize(total_surf_triangle_count);


    auto global_vertex_offset = affine_body_vertex_reporter->vertex_offset();

    // 2) for every body, build surface
    for(auto&& [body_info, body_surf_info] : zip(abd().h_body_infos, body_surface_infos))
    {
        auto  body_index = body_info.affine_body_id();
        auto  geo_index  = body_info.abd_geometry_index();
        auto& geo        = abd().geometry(geo_slots, body_info);

        if(body_surf_info.m_surf_vertex_count > 0)
        {
            auto surf_v =
                span{surf_vertices}.subspan(body_surf_info.m_surf_vertex_offset,
                                            body_surf_info.m_surf_vertex_count);

            auto geo_surf_vert_ids = span{geo_local_surf_vertices}.subspan(
                geo_surf_vertex_offsets[geo_index], geo_surf_vertex_counts[geo_index]);

            std::ranges::transform(geo_surf_vert_ids,
                                   surf_v.begin(),
                                   [&](const IndexT& geo_local_vert_id) {
                                       return geo_local_vert_id + global_vertex_offset;
                                   });
        }

        if(body_surf_info.m_surf_edge_count > 0)
        {
            auto surf_e = span{surf_edges}.subspan(body_surf_info.m_surf_edge_offset,
                                                   body_surf_info.m_surf_edge_count);

            auto geo_surf_edge_ids = span{geo_local_surf_edges}.subspan(
                geo_surf_edges_offsets[geo_index], geo_surf_edges_counts[geo_index]);

            auto Es = geo.edges().topo().view();

            std::ranges::transform(geo_surf_edge_ids,
                                   surf_e.begin(),
                                   [&](const IndexT& geo_local_edge_id) -> Vector2i
                                   {
                                       auto edge = Es[geo_local_edge_id];
                                       Vector2i ret = edge.array() + global_vertex_offset;
                                       return ret;
                                   });
        }

        if(body_surf_info.m_surf_triangle_count > 0)
        {
            auto surf_f =
                span{surf_triangles}.subspan(body_surf_info.m_surf_triangle_offset,
                                             body_surf_info.m_surf_triangle_count);

            auto geo_surf_tri_ids = span{geo_local_surf_triangles}.subspan(
                geo_surf_triangles_offsets[geo_index], geo_surf_triangles_counts[geo_index]);

            auto Fs = geo.triangles().topo().view();

            std::ranges::transform(geo_surf_tri_ids,
                                   surf_f.begin(),
                                   [&](const IndexT& geo_local_tri_id) -> Vector3i
                                   {
                                       auto tri = Fs[geo_local_tri_id];
                                       Vector3i ret = tri.array() + global_vertex_offset;
                                       return ret;
                                   });
        }

        global_vertex_offset += body_info.vertex_count();
    }
}

void AffinebodySurfaceReporter::Impl::report_count(backend::WorldVisitor& world,
                                                   GlobalSimpicialSurfaceManager::SurfaceCountInfo& info)
{
    info.surf_vertex_count(total_surf_vertex_count);
    info.surf_edge_count(total_surf_edge_count);
    info.surf_triangle_count(total_surf_triangle_count);
}

void AffinebodySurfaceReporter::Impl::report_attributes(
    backend::WorldVisitor& world, GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info)
{
    auto async_copy = []<typename T>(span<T> src, muda::BufferView<T> dst)
    { muda::BufferLaunch().copy<T>(dst, src.data()); };

    async_copy(span{surf_vertices}, info.surf_vertices());
    async_copy(span{surf_edges}, info.surf_edges());
    async_copy(span{surf_triangles}, info.surf_triangles());
}

void AffinebodySurfaceReporter::do_report_count(GlobalSimpicialSurfaceManager::SurfaceCountInfo& info)
{
    m_impl.report_count(world(), info);
}

void AffinebodySurfaceReporter::do_report_attributes(GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info)
{
    m_impl.report_attributes(world(), info);
}
}  // namespace uipc::backend::cuda
