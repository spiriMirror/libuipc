#include <finite_element/finite_element_vertex_reporter.h>
#include <global_geometry/global_vertex_manager.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <finite_element/finite_element_body_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
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
    using namespace muda;

    info.contact_element_ids().copy_from(fem().h_vertex_contact_element_ids.data());
    info.subscene_element_ids().copy_from(
        fem().h_vertex_subscene_contact_element_ids.data());

    info.dimensions().copy_from(fem().h_dimensions.data());
    info.thicknesses().copy_from(fem().thicknesses);

    info.body_ids().copy_from(fem().h_vertex_body_id.data());
    info.d_hats().copy_from(fem().h_vertex_d_hat.data());

    // fill the coindices for later use
    auto N = info.coindices().size();
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [coindices    = info.coindices().viewer().name("coindices"),
                src_pos      = fem().xs.cviewer().name("src_pos"),
                dst_pos      = info.positions().viewer().name("dst_pos"),
                src_rest_pos = fem().x_bars.cviewer().name("rest_pos"),
                body_offset  = body_reporter->body_offset(),
                dst_body_ids = info.body_ids().viewer().name("dst_body_ids"),
                dst_rest_pos = info.rest_positions().viewer().name("rest_pos")] __device__(int i) mutable
               {
                   coindices(i)    = i;
                   dst_pos(i)      = src_pos(i);
                   dst_rest_pos(i) = src_rest_pos(i);
                   dst_body_ids(i) += body_offset;  // offset by the global body offset
               });
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
