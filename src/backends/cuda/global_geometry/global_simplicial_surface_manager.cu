#include <global_geometry/global_simplicial_surface_manager.h>
#include <global_geometry/simplicial_surface_reporter.h>
#include <uipc/common/zip.h>
#include <uipc/common/enumerate.h>
#include <muda/cub/device/device_select.h>
#include <utils/offset_count_collection.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalSimplicialSurfaceManager);

void GlobalSimplicialSurfaceManager::add_reporter(SimplicialSurfaceReporter* reporter) noexcept
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    m_impl.reporters.register_sim_system(*reporter);
}

muda::CBufferView<IndexT> GlobalSimplicialSurfaceManager::codim_vertices() const noexcept
{
    return m_impl.codim_vertices;
}

muda::CBufferView<IndexT> GlobalSimplicialSurfaceManager::surf_vertices() const noexcept
{
    return m_impl.surf_vertices;
}

muda::CBufferView<Vector2i> GlobalSimplicialSurfaceManager::surf_edges() const noexcept
{
    return m_impl.surf_edges;
}

muda::CBufferView<Vector3i> GlobalSimplicialSurfaceManager::surf_triangles() const noexcept
{
    return m_impl.surf_triangles;
}

void GlobalSimplicialSurfaceManager::do_build()
{
    m_impl.global_vertex_manager = find<GlobalVertexManager>();
}

void GlobalSimplicialSurfaceManager::Impl::init()
{
    auto reporter_view = reporters.view();


    // 1) initialize the reporters
    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;


    for(auto&& R : reporter_view)
    {
        SurfaceInitInfo info;
        R->init(info);
    }

    auto reporter_count = reporter_view.size();

    // 2) compute the counts and offsets
    reporter_infos.resize(reporter_view.size());
    OffsetCountCollection<IndexT> vertex_offsets_counts;
    vertex_offsets_counts.resize(reporter_count);
    OffsetCountCollection<IndexT> edge_offsets_counts;
    edge_offsets_counts.resize(reporter_count);
    OffsetCountCollection<IndexT> triangle_offsets_counts;
    triangle_offsets_counts.resize(reporter_count);

    span<IndexT> vertex_counts   = vertex_offsets_counts.counts();
    span<IndexT> edge_counts     = edge_offsets_counts.counts();
    span<IndexT> triangle_counts = triangle_offsets_counts.counts();

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

        vertex_counts[R->m_index]   = V;
        edge_counts[R->m_index]     = E;
        triangle_counts[R->m_index] = F;
    }

    vertex_offsets_counts.scan();
    edge_offsets_counts.scan();
    triangle_offsets_counts.scan();

    span<const IndexT> vertex_offsets   = vertex_offsets_counts.offsets();
    span<const IndexT> edge_offsets     = edge_offsets_counts.offsets();
    span<const IndexT> triangle_offsets = triangle_offsets_counts.offsets();

    for(auto&& [i, Rinfo] : enumerate(reporter_infos))
    {
        Rinfo.surf_vertex_offset   = vertex_offsets[i];
        Rinfo.surf_edge_offset     = edge_offsets[i];
        Rinfo.surf_triangle_offset = triangle_offsets[i];
    }

    SizeT total_surf_vertex_count   = vertex_offsets_counts.total_count();
    SizeT total_surf_edge_count     = edge_offsets_counts.total_count();
    SizeT total_surf_triangle_count = triangle_offsets_counts.total_count();

    // 3) resize the device buffer
    codim_vertices.resize(total_surf_vertex_count);
    surf_vertices.resize(total_surf_vertex_count);
    surf_edges.resize(total_surf_edge_count);
    surf_triangles.resize(total_surf_triangle_count);


    // 4) collect surface attributes
    for(auto&& [R, Rinfo] : zip(reporter_view, reporter_infos))
    {
        SurfaceAttributeInfo info{this, R->m_index};
        R->report_attributes(info);
    }

    // 5) collect Codim0D vertices
    _collect_codim_vertices();
}

void GlobalSimplicialSurfaceManager::Impl::_collect_codim_vertices()
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
                   auto vI = surf_vertices(I);
                   flags(I) = dim(vI) <= 1 ? 1 : 0;  // codim 0D vert and vert from codim 1D edge
               });

    DeviceSelect().Flagged(surf_vertices.data(),
                           codim_vertex_flags.data(),
                           codim_vertices.data(),
                           selected_codim_0d_count.data(),
                           surf_vertices.size());

    IndexT count = selected_codim_0d_count;
    codim_vertices.resize(count);
}

void GlobalSimplicialSurfaceManager::init()
{
    m_impl.init();
}

void GlobalSimplicialSurfaceManager::rebuild()
{
    UIPC_ASSERT(false, "Not implemented yet");
}

muda::BufferView<IndexT> GlobalSimplicialSurfaceManager::SurfaceAttributeInfo::surf_vertices() noexcept
{
    const auto& info = reporter_info();
    return m_impl->surf_vertices.view(info.surf_vertex_offset, info.surf_vertex_count);
}

muda::BufferView<Vector2i> GlobalSimplicialSurfaceManager::SurfaceAttributeInfo::surf_edges() noexcept
{
    const auto& info = reporter_info();
    return m_impl->surf_edges.view(info.surf_edge_offset, info.surf_edge_count);
}

muda::BufferView<Vector3i> GlobalSimplicialSurfaceManager::SurfaceAttributeInfo::surf_triangles() noexcept
{
    const auto& info = reporter_info();
    return m_impl->surf_triangles.view(info.surf_triangle_offset, info.surf_triangle_count);
}

auto GlobalSimplicialSurfaceManager::SurfaceAttributeInfo::reporter_info() const noexcept
    -> const ReporterInfo&
{
    return m_impl->reporter_infos[m_index];
}

void GlobalSimplicialSurfaceManager::SurfaceCountInfo::surf_vertex_count(SizeT count) noexcept
{
    m_surf_vertex_count = count;
}

void GlobalSimplicialSurfaceManager::SurfaceCountInfo::surf_edge_count(SizeT count) noexcept
{
    m_surf_edge_count = count;
}

void GlobalSimplicialSurfaceManager::SurfaceCountInfo::surf_triangle_count(SizeT count) noexcept
{
    m_surf_triangle_count = count;
}

void GlobalSimplicialSurfaceManager::SurfaceCountInfo::changable(bool value) noexcept
{
    m_changable = value;
}
}  // namespace uipc::backend::cuda
