#include <global_geometry/global_simplicial_surface_manager.h>
#include <global_geometry/simplicial_surface_reporter.h>
#include <uipc/common/zip.h>
#include <uipc/common/enumerate.h>
#include <muda/cub/device/device_select.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalSimpicialSurfaceManager);

void GlobalSimpicialSurfaceManager::add_reporter(SimplicialSurfaceReporter* reporter) noexcept
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    m_impl.reporters.register_subsystem(*reporter);
}

muda::CBufferView<IndexT> GlobalSimpicialSurfaceManager::codim_vertices() const noexcept
{
    return m_impl.codim_vertices;
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
    codim_vertices.resize(total_surf_vertex_count);
    surf_vertices.resize(total_surf_vertex_count);
    surf_edges.resize(total_surf_edge_count);
    surf_triangles.resize(total_surf_triangle_count);


    // 3) collect surface attributes
    for(auto&& [R, Rinfo] : zip(reporter_view, reporter_infos))
    {
        SurfaceAttributeInfo info{this, R->m_index};
        R->report_attributes(info);
    }

    // 4) collect Codim0D vertices
    _collect_codim_vertices();
}

void GlobalSimpicialSurfaceManager::Impl::_collect_codim_vertices()
{
    using namespace muda;
    auto dim = global_vertex_manager->dimensions();

    codim_vertex_flags.resize(surf_vertices.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(surf_vertices.size(),
               [dim           = dim.cviewer().name("dim"),
                surf_vertices = surf_vertices.viewer().name("surf_vertices"),
                flags = codim_vertex_flags.viewer().name("flags")] __device__(int I) mutable
               {
                   auto vI  = surf_vertices(I);
                   flags(I) = dim(vI) <= 1 ? 1 : 0;
               });

    DeviceSelect().Flagged(surf_vertices.data(),
                           codim_vertex_flags.data(),
                           codim_vertices.data(),
                           selected_codim_0d_count.data(),
                           surf_vertices.size());

    IndexT count = selected_codim_0d_count;
    codim_vertices.resize(count);
}

void GlobalSimpicialSurfaceManager::init_surface_info()
{
    m_impl.init_surface_info();
}

void GlobalSimpicialSurfaceManager::rebuild_surface_info()
{
    UIPC_ASSERT(false, "Not implemented yet");
}

muda::BufferView<IndexT> GlobalSimpicialSurfaceManager::SurfaceAttributeInfo::surf_vertices() noexcept
{
    const auto& info = reporter_info();
    return m_impl->surf_vertices.view(info.surf_vertex_offset, info.surf_vertex_count);
}

muda::BufferView<Vector2i> GlobalSimpicialSurfaceManager::SurfaceAttributeInfo::surf_edges() noexcept
{
    const auto& info = reporter_info();
    return m_impl->surf_edges.view(info.surf_edge_offset, info.surf_edge_count);
}

muda::BufferView<Vector3i> GlobalSimpicialSurfaceManager::SurfaceAttributeInfo::surf_triangles() noexcept
{
    const auto& info = reporter_info();
    return m_impl->surf_triangles.view(info.surf_triangle_offset, info.surf_triangle_count);
}

auto GlobalSimpicialSurfaceManager::SurfaceAttributeInfo::reporter_info() const noexcept
    -> const ReporterInfo&
{
    return m_impl->reporter_infos[m_index];
}

void GlobalSimpicialSurfaceManager::SurfaceCountInfo::surf_vertex_count(SizeT count) noexcept
{
    m_surf_vertex_count = count;
}

void GlobalSimpicialSurfaceManager::SurfaceCountInfo::surf_edge_count(SizeT count) noexcept
{
    m_surf_edge_count = count;
}

void GlobalSimpicialSurfaceManager::SurfaceCountInfo::surf_triangle_count(SizeT count) noexcept
{
    m_surf_triangle_count = count;
}

void GlobalSimpicialSurfaceManager::SurfaceCountInfo::changable(bool value) noexcept
{
    m_changable = value;
}
}  // namespace uipc::backend::cuda
