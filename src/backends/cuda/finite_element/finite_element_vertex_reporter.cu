#include <finite_element/finite_element_vertex_reporter.h>
#include <global_geometry/global_vertex_manager.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <finite_element/finite_element_body_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FiniteElementVertexReporter_init_attributes_kernel(
        cuda_tool::BufferView<IndexT>    coindices,
        cuda_tool::CBufferView<Vector3>  src_pos,
        cuda_tool::BufferView<Vector3>   dst_pos,
        cuda_tool::CBufferView<Vector3>  src_rest_pos,
        IndexT                           body_offset,
        cuda_tool::BufferView<IndexT>    dst_body_ids,
        cuda_tool::BufferView<Vector3>   dst_rest_pos,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        coindices(i)    = i;
        dst_pos(i)      = src_pos(i);
        dst_rest_pos(i) = src_rest_pos(i);
        dst_body_ids(i) += body_offset;  // offset by the global body offset
    }
}  // namespace

REGISTER_SIM_SYSTEM(FiniteElementVertexReporter);

constexpr static U64 FiniteElementVertexReporterUID = 1;

void FiniteElementVertexReporter::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
    m_impl.body_reporter         = &require<FiniteElementBodyReporter>();
}

void FiniteElementVertexReporter::request_attribute_update() noexcept
{
    m_impl.require_update_attributes = true;
}

void FiniteElementVertexReporter::Impl::report_count(VertexCountInfo& info)
{
    info.count(fem().xs.size());
}

void FiniteElementVertexReporter::Impl::init_attributes(VertexAttributeInfo& info)
{
    info.contact_element_ids().copy_from(fem().h_vertex_contact_element_ids.data());
    info.subscene_element_ids().copy_from(
        fem().h_vertex_subscene_contact_element_ids.data());

    info.dimensions().copy_from(fem().h_dimensions.data());
    info.thicknesses().copy_from(fem().thicknesses);

    info.body_ids().copy_from(fem().h_vertex_body_id.data());
    info.d_hats().copy_from(fem().h_vertex_d_hat.data());

    // fill the coindices for later use
    auto k = FiniteElementVertexReporter_init_attributes_kernel;
    int  n = (int)info.coindices().size();
    if(n > 0)
    {
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.coindices(),
            fem().xs.cview(),
            info.positions(),
            fem().x_bars.cview(),
            body_reporter->body_offset(),
            info.body_ids(),
            info.rest_positions(),
            n);
    }
}

void FiniteElementVertexReporter::Impl::update_attributes(VertexAttributeInfo& info)
{
    info.positions().copy_from(fem().xs);

    // This update will ruin the friction force computed in previous step, so we need to discard it.
    // ref: https://github.com/spiriMirror/libuipc/issues/303
    info.require_discard_friction();
}

void FiniteElementVertexReporter::Impl::report_displacements(VertexDisplacementInfo& info)
{
    info.displacements().copy_from(fem().dxs);
}

void FiniteElementVertexReporter::do_report_count(VertexCountInfo& info)
{
    m_impl.report_count(info);
}

void FiniteElementVertexReporter::do_report_attributes(VertexAttributeInfo& info)
{
    if(info.frame() == 0)
    {
        auto global_offset = info.coindices().offset();

        auto geo_slots = world().scene().geometries();

        // add global vertex offset attribute
        m_impl.finite_element_method->for_each(  //
            geo_slots,
            [&](const FiniteElementMethod::ForEachInfo& I, geometry::SimplicialComplex& sc)
            {
                auto gvo = sc.meta().find<IndexT>(builtin::global_vertex_offset);
                if(!gvo)
                {
                    gvo = sc.meta().create<IndexT>(builtin::global_vertex_offset);
                }

                // [global-vertex-offset] = [vertex-offset-in-fem-system] + [fem-system-vertex-offset]
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

void FiniteElementVertexReporter::do_report_displacements(VertexDisplacementInfo& info)
{
    m_impl.report_displacements(info);
}

U64 FiniteElementVertexReporter::get_uid() const noexcept
{
    return FiniteElementVertexReporterUID;
}
}  // namespace uipc::backend::cuda
