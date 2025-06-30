#include <affine_body/affine_body_vertex_reporter.h>
#include <global_geometry/global_vertex_manager.h>
#include <affine_body/affine_body_body_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyVertexReporter);

void AffineBodyVertexReporter::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
    m_impl.body_reporter        = &require<AffineBodyBodyReporter>();
}

void AffineBodyVertexReporter::Impl::report_count(VertexCountInfo& info)
{
    // TODO: now we just report all the affine body vertices
    // later we may extract the surface vertices and report them
    info.count(abd().h_vertex_id_to_J.size());
}

void AffineBodyVertexReporter::Impl::report_attributes(VertexAttributeInfo& info)
{
    using namespace muda;
    // fill the coindices for later use
    auto N = info.coindices().size();

    UIPC_ASSERT(body_reporter->body_offset() >= 0,
                "AffineBodyBodyReporter is not ready, body_offset={}, lifecycle issue?",
                body_reporter->body_offset());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [coindices   = info.coindices().viewer().name("coindices"),
                src_pos     = abd().vertex_id_to_J.cviewer().name("src_pos"),
                dst_pos     = info.positions().viewer().name("dst_pos"),
                v2b         = abd().vertex_id_to_body_id.cviewer().name("v2b"),
                body_offset = body_reporter->body_offset(),
                dst_v2b     = info.body_ids().viewer().name("dst_v2b"),
                qs          = abd().body_id_to_q.cviewer().name("qs"),
                dst_rest_pos = info.rest_positions().viewer().name("rest_pos")] __device__(int i) mutable
               {
                   coindices(i) = i;

                   auto        body_id = v2b(i);
                   const auto& q       = qs(body_id);
                   dst_pos(i)          = src_pos(i).point_x(q);
                   dst_rest_pos(i)     = src_pos(i).x_bar();
                   dst_v2b(i) = body_id + body_offset;  // offset by the global body offset
               });

    auto async_copy = []<typename T>(span<T> src, muda::BufferView<T> dst)
    { muda::BufferLaunch().copy<T>(dst, src.data()); };

    async_copy(span{abd().h_vertex_id_to_contact_element_id}, info.contact_element_ids());
}

void AffineBodyVertexReporter::Impl::report_displacements(VertexDisplacementInfo& info)
{
    using namespace muda;
    auto N = info.coindices().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(N,
               [coindices = info.coindices().viewer().name("coindices"),
                displacements = info.displacements().viewer().name("displacements"),
                v2b = abd().vertex_id_to_body_id.cviewer().name("v2b"),
                dqs = abd().body_id_to_dq.cviewer().name("dqs"),
                Js = abd().vertex_id_to_J.cviewer().name("Js")] __device__(int vI) mutable
               {
                   auto             body_id = v2b(vI);
                   const Vector12&  dq      = dqs(body_id);
                   const ABDJacobi& J       = Js(vI);
                   auto&            dx      = displacements(vI);
                   dx                       = J * dq;
               });
}

void AffineBodyVertexReporter::do_report_count(VertexCountInfo& info)
{
    m_impl.report_count(info);
}
void AffineBodyVertexReporter::do_report_attributes(VertexAttributeInfo& info)
{
    m_impl.report_attributes(info);


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
}

void AffineBodyVertexReporter::do_report_displacements(VertexDisplacementInfo& info)
{
    m_impl.report_displacements(info);
}
}  // namespace uipc::backend::cuda
