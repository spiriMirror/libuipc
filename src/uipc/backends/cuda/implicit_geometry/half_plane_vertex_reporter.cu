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
    //using namespace muda;
    //// fill the coindices for later use
    //auto N = info.coindices().size();

    //ParallelFor()
    //    .kernel_name(__FUNCTION__)
    //    .apply(N,
    //           [coindices = info.coindices().viewer().name("coindices"),

    //            dst_pos = info.positions().viewer().name("dst_pos"),
    //            src_pos = half_plane->m_impl.positions.viewer().name("src_pos")] __device__(int i) mutable
    //           {
    //               coindices(i) = i;
    //               dst_pos(i)   = src_pos(i);
    //           });

    //auto async_copy = []<typename T>(span<T> src, muda::BufferView<T> dst)
    //{ muda::BufferLaunch().copy<T>(dst, src.data()); };

    //async_copy(span{abd().h_vertex_id_to_contact_element_id}, info.contact_element_ids());

    //// record the global vertex info
    //reporter_vertex_offset = info.coindices().offset();
    //reporter_vertex_count  = info.coindices().size();
}
void HalfPlaneVertexReporter::Impl::report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
{
}

void HalfPlaneVertexReporter::do_report_count(GlobalVertexManager::VertexCountInfo& info)
{
    info.count(m_impl.half_plane->m_impl.h_positions.size());
}

void HalfPlaneVertexReporter::do_report_attributes(GlobalVertexManager::VertexAttributeInfo& vertex_attribute_info)
{
}

void HalfPlaneVertexReporter::do_report_displacements(GlobalVertexManager::VertexDisplacementInfo& vertex_displacement_info)
{
}
}  // namespace uipc::backend::cuda
