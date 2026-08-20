#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <implicit_geometry/half_plane.h>
#include <implicit_geometry/half_plane_body_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void HalfPlaneVertexReporter_report_attributes_kernel(
        cuda_tool::BufferView<IndexT>  coindices,
        cuda_tool::BufferView<Vector3> dst_pos,
        cuda_tool::BufferView<Vector3> src_pos,
        cuda_tool::BufferView<IndexT>  dst_vertex_body_ids,
        IndexT                         body_offset,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        coindices(i) = i;
        dst_pos(i)   = src_pos(i);
        // each vertex corresponds to a body
        // so we can use the body offset + i to get global body id
        dst_vertex_body_ids(i) = body_offset + i;
    }
}  // namespace

REGISTER_SIM_SYSTEM(HalfPlaneVertexReporter);

constexpr U64 HalfPlaneVertexReporterUID = 2;

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
    using namespace cuda_tool;
    // fill the coindices for later use
    auto N = info.coindices().size();

    auto k = HalfPlaneVertexReporter_report_attributes_kernel;
    int  n = (int)N;
    if(n > 0)
    {
        k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
            info.coindices().viewer(),
            info.positions().viewer(),
            half_plane->m_impl.positions.viewer(),
            info.body_ids().viewer(),
            body_reporter->body_offset(),
            n);
    }

    info.contact_element_ids().copy_from(half_plane->m_impl.h_contact_ids.data());
    info.subscene_element_ids().copy_from(half_plane->m_impl.h_subscene_ids.data());
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

U64 HalfPlaneVertexReporter::get_uid() const noexcept
{
    return HalfPlaneVertexReporterUID;
}
}  // namespace uipc::backend::cuda
