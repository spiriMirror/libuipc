#include <implicit_geometry/half_plane_body_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(HalfPlaneBodyReporter);

void HalfPlaneBodyReporter::do_build(BuildInfo& info)
{
    m_impl.half_plane = &require<HalfPlane>();
}

void HalfPlaneBodyReporter::do_init(InitInfo& info) {}

void HalfPlaneBodyReporter::Impl::report_count(BodyCountInfo& info)
{
    // One position and one normal per half plane body
    // so body_count is equal to the position count.
    auto body_count = half_plane->m_impl.h_positions.size();
    info.count(body_count);
}

void HalfPlaneBodyReporter::Impl::report_attributes(BodyAttributeInfo& info)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(info.coindices().size(),
               [coindices = info.coindices().viewer().name("coindices")] __device__(int i)
               {
                   coindices(i) = i;  // just iota
               });

    // HalfPlane does not have self-collision, so we fill it with zeros.
    info.self_collision().fill(0);
}

void HalfPlaneBodyReporter::do_report_count(BodyCountInfo& info)
{
    m_impl.report_count(info);
}

void HalfPlaneBodyReporter::do_report_attributes(BodyAttributeInfo& info)
{
    m_impl.report_attributes(info);
}
}  // namespace uipc::backend::cuda
