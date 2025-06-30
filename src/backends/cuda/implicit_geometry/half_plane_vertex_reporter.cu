#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <implicit_geometry/half_plane.h>
#include <implicit_geometry/half_plane_body_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(HalfPlaneVertexReporter);

void HalfPlaneVertexReporter::do_build(BuildInfo& info)
{
    m_impl.half_plane    = &require<HalfPlane>();
    m_impl.body_reporter = &require<HalfPlaneBodyReporter>();
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

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [coindices = info.coindices().viewer().name("coindices"),
                dst_pos   = info.positions().viewer().name("dst_pos"),
                src_pos = half_plane->m_impl.positions.viewer().name("src_pos"),
                dst_vertex_body_ids = info.body_ids().viewer().name("dst_vertex_body_ids"),
                body_offset = body_reporter->body_offset()] __device__(int i) mutable
               {
                   coindices(i) = i;
                   dst_pos(i)   = src_pos(i);
                   // each vertex corresponds to a body
                   // so we can use the body offset + i to get global body id
                   dst_vertex_body_ids(i) = body_offset + i;
               });

    info.contact_element_ids().copy_from(half_plane->m_impl.h_contact_ids.data());
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


    auto global_offset = info.coindices().offset();

    auto geo_slots = world().scene().geometries();

    // add global vertex offset attribute
    m_impl.half_plane->for_each(  //
        geo_slots,
        [&](const HalfPlane::ForEachInfo& I, geometry::ImplicitGeometry& ig)
        {
            auto gvo = ig.meta().find<IndexT>(builtin::global_vertex_offset);
            if(!gvo)
            {
                gvo = ig.meta().create<IndexT>(builtin::global_vertex_offset);
            }

            // [global-vertex-offset] = [vertex-offset-in-halfplane-system] + [halfplane-system-vertex-offset]
            view(*gvo)[0] = I.geo_info().vertex_offset + global_offset;
        });
}

void HalfPlaneVertexReporter::do_report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
{
    m_impl.report_displacements(info);
}
}  // namespace uipc::backend::cuda
