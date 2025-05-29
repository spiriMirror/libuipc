#include <affine_body/affine_body_body_reporter.h>
#include <muda/launch/parallel_for.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyBodyReporter);

void AffineBodyBodyReporter::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
}

void AffineBodyBodyReporter::do_init(InitInfo& info)
{
    // do nothing
}

void AffineBodyBodyReporter::do_report_count(BodyCountInfo& info)
{
    m_impl.report_count(info);
}

void AffineBodyBodyReporter::do_report_attributes(BodyAttributeInfo& info)
{
    m_impl.report_attributes(info);
}

void AffineBodyBodyReporter::Impl::report_count(BodyCountInfo& info)
{
    auto N = affine_body_dynamics->m_impl.body_count();
    info.count(N);
    info.changeable(false);
}

void AffineBodyBodyReporter::Impl::report_attributes(BodyAttributeInfo& info)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(info.coindices().size(),
               [coindices = info.coindices().viewer().name("coindices")] __device__(int i)
               {
                   coindices(i) = i;  // just iota
               });

    span<const IndexT> self_collision = affine_body_dynamics->m_impl.h_body_id_to_self_collision;

    UIPC_ASSERT(self_collision.size() == info.self_collision().size(),
                "Size mismatch in self-collision data, info size: {}, self_collision size: {}",
                info.self_collision().size(),
                self_collision.size());

    info.self_collision().copy_from(self_collision.data());
}
}  // namespace uipc::backend::cuda