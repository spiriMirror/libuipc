#include <global_surface_manager.h>
#include <surface_reporter.h>
#include <uipc/common/zip.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalSurfaceManager);

void GlobalSurfaceManager::add_reporter(SurfaceReporter* reporter) noexcept
{
    reporter->m_index = m_impl.reporter_buffer.size();
    m_impl.reporter_buffer.push_back(reporter);
}

void GlobalSurfaceManager::do_build()
{
    m_impl.global_vertex_manager = find<GlobalVertexManager>();
}

void GlobalSurfaceManager::Impl::init_surface_info()
{
    // 1) build the core invariant data structure: reporter_infos
    reporters.resize(reporter_buffer.size());
    std::ranges::move(reporter_buffer, reporters.begin());

    reporter_infos.resize(reporters.size());
    vector<SizeT> vertex_offsets(reporters.size(), 0);
    vector<SizeT> edge_offsets(reporters.size(), 0);
    vector<SizeT> triangle_offsets(reporters.size(), 0);
    for(auto&& [R, Rinfo] : zip(reporters, reporter_infos))
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
    auto& back_info = reporter_infos.back();
    total_surf_vertex_count = back_info.surf_vertex_offset + back_info.surf_vertex_count;
    total_surf_edge_count = back_info.surf_edge_offset + back_info.surf_edge_count;
    total_surf_triangle_count = back_info.surf_triangle_offset + back_info.surf_triangle_count;

    // 2) resize the device buffer
    surf_vertices.resize(total_surf_vertex_count);
    // init as default contact element id
    contact_element_ids.resize(total_surf_vertex_count, 0);

    surf_edges.resize(total_surf_edge_count);
    surf_triangles.resize(total_surf_triangle_count);


    // 3) collect surface attributes
    for(auto&& [R, Rinfo] : zip(reporters, reporter_infos))
    {
        SurfaceAttributeInfo info{this};

        info.m_surf_vertices =
            surf_vertices.view(Rinfo.surf_vertex_offset, Rinfo.surf_vertex_count);
        info.m_contact_element_ids =
            contact_element_ids.view(Rinfo.surf_vertex_offset, Rinfo.surf_vertex_count);
        info.m_surf_edges = surf_edges.view(Rinfo.surf_edge_offset, Rinfo.surf_edge_count);
        info.m_surf_triangles =
            surf_triangles.view(Rinfo.surf_triangle_offset, Rinfo.surf_triangle_count);

        R->report_attributes(info);
    }

    {
        std::vector<IndexT>   surf_vertices;
        std::vector<Vector2i> surf_edges;
        std::vector<Vector3i> surf_triangles;

        this->surf_vertices.copy_to(surf_vertices);
        this->surf_edges.copy_to(surf_edges);
        this->surf_triangles.copy_to(surf_triangles);

        // print
        std::cout << "surf_vertices: " << std::endl;
        for(auto&& v : surf_vertices)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        std::cout << "surf_edges: " << std::endl;
        for(auto&& e : surf_edges)
        {
            std::cout << e.transpose() << " ";
        }
        std::cout << std::endl;

        std::cout << "surf_triangles: " << std::endl;
        for(auto&& t : surf_triangles)
        {
            std::cout << t.transpose() << " ";
        }
        std::cout << std::endl;
    }
}

void GlobalSurfaceManager::init_surface_info()
{
    m_impl.init_surface_info();
}

void GlobalSurfaceManager::rebuild_surface_info()
{
    UIPC_ASSERT(false, "Not implemented yet");
}
}  // namespace uipc::backend::cuda
