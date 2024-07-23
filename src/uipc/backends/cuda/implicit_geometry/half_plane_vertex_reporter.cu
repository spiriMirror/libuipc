#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <implicit_geometry/half_plane.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(HalfPlaneVertexReporter);

void HalfPlaneVertexReporter::do_build()
{
    m_impl.half_plane = &require<HalfPlane>();
}

void HalfPlaneVertexReporter::Impl::report_count(GlobalVertexManager::VertexCountInfo& info)
{
    info.count(half_plane->m_impl.h_positions.size());
}

void HalfPlaneVertexReporter::Impl::report_attributes(GlobalVertexManager::VertexAttributeInfo& info)
{
    using namespace muda;
    // fill the coindices for later use
    auto N = info.coindices().size();

    vertex_global_offset = info.coindices().offset();
    vertex_count         = info.coindices().size();

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(N,
               [coindices = info.coindices().viewer().name("coindices"),

                dst_pos = info.positions().viewer().name("dst_pos"),
                src_pos = half_plane->m_impl.positions.viewer().name("src_pos")] __device__(int i) mutable
               {
                   coindices(i) = i;
                   dst_pos(i)   = src_pos(i);
               });

    info.contact_element_ids().copy_from(half_plane->m_impl.h_contact_id.data());
}

void HalfPlaneVertexReporter::Impl::report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
{
    // Now, we only support fixed half plane
    info.displacements().fill(Vector3::Zero());
}

void HalfPlaneVertexReporter::do_report_count(GlobalVertexManager::VertexCountInfo& info)
{
    m_impl.report_count(info);
}

void HalfPlaneVertexReporter::do_report_attributes(GlobalVertexManager::VertexAttributeInfo& info)
{
    m_impl.report_attributes(info);
}

void HalfPlaneVertexReporter::do_report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
{
    m_impl.report_displacements(info);
}

size_t HalfPlaneVertexReporter::vertex_global_offset() const
{
    return m_impl.vertex_global_offset;
}

size_t HalfPlaneVertexReporter::vertex_count() const
{
    return m_impl.vertex_count;
}
}  // namespace uipc::backend::cuda
