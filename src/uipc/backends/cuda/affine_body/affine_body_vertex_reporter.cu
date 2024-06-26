#include <affine_body/affine_body_vertex_reporter.h>
#include <global_vertex_manager.h>
namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<AffineBodyVertexReporter>
{
  public:
    static U<AffineBodyVertexReporter> create(SimEngine& engine)
    {
        return has_affine_body_constitution(engine) ?
                   make_unique<AffineBodyVertexReporter>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(AffineBodyVertexReporter);

void AffineBodyVertexReporter::do_build()
{
    m_impl.affine_body_dynamics = find<AffineBodyDynamics>();
    auto global_vertex_manager  = find<GlobalVertexManager>();
    global_vertex_manager->add_reporter(this);
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
    // fill the coindex for later use
    auto N = info.coindex().size();
    // TODO: now we just use `iota` the coindex
    // later we may extract the surface vertices as the reported vertices
    // then the coindex will be a mapping from the surface vertices to the affine body vertices
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(N,
               [coindex = info.coindex().viewer().name("coindex"),

                dst_pos = info.positions().viewer().name("dst_pos"),
                v2b     = abd().vertex_id_to_body_id.cviewer().name("v2b"),
                qs      = abd().body_id_to_q.cviewer().name("qs"),
                src_pos = abd().vertex_id_to_J.cviewer().name("src_pos")] __device__(int i) mutable
               {
                   coindex(i) = i;

                   auto        body_id = v2b(i);
                   const auto& q       = qs(body_id);
                   dst_pos(i)          = src_pos(i).point_x(q);
               });

    // record the global vertex info
    reporter_vertex_offset = info.coindex().offset();
    reporter_vertex_count  = info.coindex().size();
}

void AffineBodyVertexReporter::Impl::report_displacements(GlobalVertexManager::VertexDisplacementInfo& info)
{
    using namespace muda;
    auto N = info.coindex().size();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(N,
               [coindex = info.coindex().viewer().name("coindex"),
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
