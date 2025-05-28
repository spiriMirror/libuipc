#include <finite_element/finite_element_body_reporter.h>
#include <muda/launch/parallel_for.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FiniteElementBodyReporter);

void FiniteElementBodyReporter::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
}

void FiniteElementBodyReporter::do_init(InitInfo& info)
{
    // do nothing
}

void FiniteElementBodyReporter::do_report_count(BodyCountInfo& info)
{
    m_impl.report_count(info);
}

void FiniteElementBodyReporter::do_report_attributes(BodyAttributeInfo& info)
{
    m_impl.report_attributes(info);
}

void FiniteElementBodyReporter::Impl::report_count(BodyCountInfo& info)
{
    auto N = finite_element_method->m_impl.h_body_self_collision.size();
    info.count(N);
    info.changeable(false);
}

void FiniteElementBodyReporter::Impl::report_attributes(BodyAttributeInfo& info)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(info.coindices().size(),
               [coindices = info.coindices().viewer().name("coindices")] __device__(int i)
               {
                   coindices(i) = i;  // just iota
               });

    span<const IndexT> self_collision = finite_element_method->m_impl.h_body_self_collision;

    UIPC_ASSERT(self_collision.size() == info.self_collision().size(),
                "Size mismatch in self-collision data, info size: {}, self_collision size: {}",
                info.self_collision().size(),
                self_collision.size());

    info.self_collision().copy_from(self_collision.data());
}
}  // namespace uipc::backend::cuda