#include <affine_body/affine_body_vertex_reporter.h>
#include <global_geometry/global_vertex_manager.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyVertexReporter);

void AffineBodyVertexReporter::do_build()
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
    auto& global_vertex_manager = require<GlobalVertexManager>();
    global_vertex_manager.add_reporter(this);
}

void AffineBodyVertexReporter::Impl::report_count(GlobalVertexManager::VertexCountInfo& info)
{
    // TODO: now we just report all the affine body vertices
    // later we may extract the surface vertices and report them
    info.count(abd().h_vertex_id_to_J.size());
}

void AffineBodyVertexReporter::Impl::report_attributes(GlobalVertexManager::VertexAttributeInfo& info)
{
    using namespace muda;
    // fill the coindices for later use
    auto N = info.coindices().size();
    // TODO: now we just use `iota` the coindices
    // later we may extract the surface vertices as the reported vertices
    // then the coindices will be a mapping from the surface vertices to the affine body vertices
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(N,
               [coindices = info.coindices().viewer().name("coindices"),

                dst_pos = info.positions().viewer().name("dst_pos"),
                v2b     = abd().vertex_id_to_body_id.cviewer().name("v2b"),
                qs      = abd().body_id_to_q.cviewer().name("qs"),
                src_pos = abd().vertex_id_to_J.cviewer().name("src_pos")] __device__(int i) mutable
               {
                   coindices(i) = i;

                   auto        body_id = v2b(i);
                   const auto& q       = qs(body_id);
                   dst_pos(i)          = src_pos(i).point_x(q);
               });

    auto async_copy = []<typename T>(span<T> src, muda::BufferView<T> dst)
    { muda::BufferLaunch().copy<T>(dst, src.data()); };

    async_copy(span{abd().h_vertex_id_to_contact_element_id}, info.contact_element_ids());

    // record the global vertex info
    reporter_vertex_offset = info.coindices().offset();
    reporter_vertex_count  = info.coindices().size();
}

void AffineBodyVertexReporter::Impl::report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
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

void AffineBodyVertexReporter::do_report_count(GlobalVertexManager::VertexCountInfo& info)
{
    m_impl.report_count(info);
}
void AffineBodyVertexReporter::do_report_attributes(GlobalVertexManager::VertexAttributeInfo& info)
{
    m_impl.report_attributes(info);
}

void AffineBodyVertexReporter::do_report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
{
    m_impl.report_displacements(info);
}
}  // namespace uipc::backend::cuda
