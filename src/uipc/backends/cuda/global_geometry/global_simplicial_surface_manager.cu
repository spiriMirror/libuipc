#include <global_geometry/global_simplicial_surface_manager.h>
#include <global_geometry/simplicial_surface_reporter.h>
#include <uipc/common/zip.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalSimpicialSurfaceManager);

void GlobalSimpicialSurfaceManager::add_reporter(SimplicialSurfaceReporter* reporter) noexcept
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    m_impl.reporters.register_subsystem(*reporter);
}

muda::CBufferView<IndexT> GlobalSimpicialSurfaceManager::surf_vertices() const noexcept
{
    return m_impl.surf_vertices;
}

muda::CBufferView<Vector2i> GlobalSimpicialSurfaceManager::surf_edges() const noexcept
{
    return m_impl.surf_edges;
}

muda::CBufferView<Vector3i> GlobalSimpicialSurfaceManager::surf_triangles() const noexcept
{
    return m_impl.surf_triangles;
}

void GlobalSimpicialSurfaceManager::do_build()
{
    m_impl.global_vertex_manager = find<GlobalVertexManager>();
}

void GlobalSimpicialSurfaceManager::Impl::init_surface_info()
{
    // 1) build the core invariant data structure: reporter_infos
    auto reporter_view = reporters.view();
    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;

    reporter_infos.resize(reporter_view.size());
    vector<SizeT> vertex_offsets(reporter_view.size(), 0);
    vector<SizeT> edge_offsets(reporter_view.size(), 0);
    vector<SizeT> triangle_offsets(reporter_view.size(), 0);
    for(auto&& [R, Rinfo] : zip(reporter_view, reporter_infos))
    {
        SurfaceCountInfo info;
        R->report_count(info);
        auto V = info.m_surf_vertex_count;
        auto E = info.m_surf_edge_count;
        auto F = info.m_surf_triangle_count;

        Rinfo.surf_vertex_count   = V;
        Rinfo.surf_edge_count     = E;
        Rinfo.surf_triangle_count = F;

        vertex_offsets[R->m_index]   = V;
        edge_offsets[R->m_index]     = E;
        triangle_offsets[R->m_index] = F;
    }

    std::exclusive_scan(
        vertex_offsets.begin(), vertex_offsets.end(), vertex_offsets.begin(), 0);
    std::exclusive_scan(edge_offsets.begin(), edge_offsets.end(), edge_offsets.begin(), 0);
    std::exclusive_scan(
        triangle_offsets.begin(), triangle_offsets.end(), triangle_offsets.begin(), 0);

    for(auto&& [i, Rinfo] : enumerate(reporter_infos))
    {
        Rinfo.surf_vertex_offset   = vertex_offsets[i];
        Rinfo.surf_edge_offset     = edge_offsets[i];
        Rinfo.surf_triangle_offset = triangle_offsets[i];
    }

    // set related data
    SizeT total_surf_vertex_count   = 0;
    SizeT total_surf_edge_count     = 0;
    SizeT total_surf_triangle_count = 0;

    if(!reporter_infos.empty())
    {
        auto& back_info = reporter_infos.back();
        total_surf_vertex_count = back_info.surf_vertex_offset + back_info.surf_vertex_count;
        total_surf_edge_count = back_info.surf_edge_offset + back_info.surf_edge_count;
        total_surf_triangle_count = back_info.surf_triangle_offset + back_info.surf_triangle_count;
    }

    // 2) resize the device buffer
    surf_vertices.resize(total_surf_vertex_count);
    surf_edges.resize(total_surf_edge_count);
    surf_triangles.resize(total_surf_triangle_count);


    // 3) collect surface attributes
    for(auto&& [R, Rinfo] : zip(reporter_view, reporter_infos))
    {
        SurfaceAttributeInfo info{this};

        info.m_surf_vertices =
            surf_vertices.view(Rinfo.surf_vertex_offset, Rinfo.surf_vertex_count);
        info.m_surf_edges = surf_edges.view(Rinfo.surf_edge_offset, Rinfo.surf_edge_count);
        info.m_surf_triangles =
            surf_triangles.view(Rinfo.surf_triangle_offset, Rinfo.surf_triangle_count);

        R->report_attributes(info);
    }

    //{
    //    std::vector<IndexT>   surf_vertices;
    //    std::vector<Vector2i> surf_edges;
    //    std::vector<Vector3i> surf_triangles;

    //    this->surf_vertices.copy_to(surf_vertices);
    //    this->surf_edges.copy_to(surf_edges);
    //    this->surf_triangles.copy_to(surf_triangles);

    //    // print
    //    std::cout << "surf_vertices: " << std::endl;
    //    for(auto&& v : surf_vertices)
    //    {
    //        std::cout << v << " ";
    //    }
    //    std::cout << std::endl;

    //    std::cout << "surf_edges: " << std::endl;
    //    for(auto&& e : surf_edges)
    //    {
    //        std::cout << e.transpose() << " ";
    //    }
    //    std::cout << std::endl;

    //    std::cout << "surf_triangles: " << std::endl;
    //    for(auto&& t : surf_triangles)
    //    {
    //        std::cout << t.transpose() << " ";
    //    }
    //    std::cout << std::endl;
    //}
}

void GlobalSimpicialSurfaceManager::init_surface_info()
{
    m_impl.init_surface_info();
}

void GlobalSimpicialSurfaceManager::rebuild_surface_info()
{
    UIPC_ASSERT(false, "Not implemented yet");
}
}  // namespace uipc::backend::cuda
