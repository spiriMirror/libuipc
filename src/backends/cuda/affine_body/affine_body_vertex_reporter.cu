#include <affine_body/affine_body_vertex_reporter.h>
#include <global_geometry/global_vertex_manager.h>
#include <affine_body/affine_body_body_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void affine_body_vertex_reporter_init_attributes_kernel(
        cuda_tool::BufferView<IndexT>     coindices,
        cuda_tool::CBufferView<ABDJacobi> src_pos,
        cuda_tool::BufferView<Vector3>    dst_pos,
        cuda_tool::CBufferView<IndexT>    v2b,
        IndexT                            body_offset,
        cuda_tool::BufferView<IndexT>     dst_v2b,
        cuda_tool::CBufferView<Vector12>  qs,
        cuda_tool::BufferView<Vector3>    dst_rest_pos,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        coindices(i) = i;

        auto        body_id = v2b(i);
        const auto& q       = qs(body_id);
        dst_pos(i)          = src_pos(i).point_x(q);
        dst_rest_pos(i)     = src_pos(i).x_bar();
        dst_v2b(i) = body_id + body_offset;  // offset by the global body offset
    }

    __global__ void affine_body_vertex_reporter_update_attributes_kernel(
        cuda_tool::BufferView<IndexT>     coindices,
        cuda_tool::CBufferView<ABDJacobi> src_pos,
        cuda_tool::BufferView<Vector3>    dst_pos,
        cuda_tool::CBufferView<IndexT>    v2b,
        cuda_tool::CBufferView<Vector12>  qs,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        coindices(i)        = i;
        auto        body_id = v2b(i);
        const auto& q       = qs(body_id);
        dst_pos(i)          = src_pos(i).point_x(q);
    }

    __global__ void affine_body_vertex_reporter_report_displacements_kernel(
        cuda_tool::BufferView<Vector3>    displacements,
        cuda_tool::CBufferView<IndexT>    v2b,
        cuda_tool::CBufferView<Vector12>  dqs,
        cuda_tool::CBufferView<ABDJacobi> Js,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto             body_id = v2b(i);
        const Vector12&  dq      = dqs(body_id);
        const ABDJacobi& J       = Js(i);
        auto&            dx      = displacements(i);
        dx                       = J * dq;
    }
}  // namespace

REGISTER_SIM_SYSTEM(AffineBodyVertexReporter);

constexpr static U64 AffineBodyVertexReporterUID = 0;

void AffineBodyVertexReporter::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
    m_impl.body_reporter        = &require<AffineBodyBodyReporter>();
}

void AffineBodyVertexReporter::request_attribute_update() noexcept
{
    m_impl.require_update_attributes = true;
}

void AffineBodyVertexReporter::Impl::report_count(VertexCountInfo& info)
{
    info.count(abd().h_vertex_id_to_J.size());
}

void AffineBodyVertexReporter::Impl::init_attributes(VertexAttributeInfo& info)
{
    auto N = info.positions().size();


    UIPC_ASSERT(body_reporter->body_offset() >= 0,
                "AffineBodyBodyReporter is not ready, body_offset={}, lifecycle issue?",
                body_reporter->body_offset());

    int n = (int)N;
    if(n > 0)
    {
        auto k = affine_body_vertex_reporter_init_attributes_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.coindices(),
            abd().vertex_id_to_J.cview(),
            info.positions(),
            abd().vertex_id_to_body_id.cview(),
            body_reporter->body_offset(),
            info.body_ids(),
            abd().body_id_to_q.cview(),
            info.rest_positions(),
            n);
    }

    auto async_copy = []<typename T>(span<T> src, cuda_tool::BufferView<T> dst)
    { cuda_tool::BufferLaunch().copy<T>(dst, src.data()); };

    async_copy(span{abd().h_vertex_id_to_contact_element_id}, info.contact_element_ids());
    async_copy(span{abd().h_vertex_id_to_subscene_contact_element_id},
               info.subscene_element_ids());
    async_copy(span{abd().h_vertex_id_to_d_hat}, info.d_hats());
}

void AffineBodyVertexReporter::Impl::update_attributes(VertexAttributeInfo& info)
{
    auto N = info.positions().size();

    // only update positions
    int n = (int)N;
    if(n > 0)
    {
        auto k = affine_body_vertex_reporter_update_attributes_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.coindices(),
            abd().vertex_id_to_J.cview(),
            info.positions(),
            abd().vertex_id_to_body_id.cview(),
            abd().body_id_to_q.cview(),
            n);
    }

    // This update will ruin the friction force computed in previous step, so we need to discard it.
    // ref: https://github.com/spiriMirror/libuipc/issues/303
    info.require_discard_friction();
}

void AffineBodyVertexReporter::Impl::report_displacements(VertexDisplacementInfo& info)
{
    auto N = info.coindices().size();
    int  n = (int)N;
    if(n > 0)
    {
        auto k = affine_body_vertex_reporter_report_displacements_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.displacements(),
            abd().vertex_id_to_body_id.cview(),
            abd().body_id_to_dq.cview(),
            abd().vertex_id_to_J.cview(),
            n);
    }
}

void AffineBodyVertexReporter::do_report_count(VertexCountInfo& info)
{
    m_impl.report_count(info);
}

void AffineBodyVertexReporter::do_report_attributes(VertexAttributeInfo& info)
{
    if(info.frame() == 0)
    {
        auto global_offset = info.coindices().offset();

        auto geo_slots = world().scene().geometries();

        // add global vertex offset attribute
        m_impl.affine_body_dynamics->for_each(  //
            geo_slots,
            [&](const AffineBodyDynamics::ForEachInfo& I, geometry::SimplicialComplex& sc)
            {
                auto gvo = sc.meta().find<IndexT>(builtin::global_vertex_offset);
                if(!gvo)
                {
                    gvo = sc.meta().create<IndexT>(builtin::global_vertex_offset);
                }

                // [global-vertex-offset] = [vertex-offset-in-abd-system] + [abd-system-vertex-offset]
                view(*gvo)[0] = I.geo_info().vertex_offset + global_offset;
            });

        m_impl.init_attributes(info);
    }
    else
    {
        if(m_impl.require_update_attributes)
        {
            m_impl.update_attributes(info);
            m_impl.require_update_attributes = false;
        }
    }
}

void AffineBodyVertexReporter::do_report_displacements(VertexDisplacementInfo& info)
{
    m_impl.report_displacements(info);
}

U64 AffineBodyVertexReporter::get_uid() const noexcept
{
    return AffineBodyVertexReporterUID;
}
}  // namespace uipc::backend::cuda
