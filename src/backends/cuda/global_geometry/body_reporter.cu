#include <global_geometry/body_reporter.h>

namespace uipc::backend::cuda
{
void BodyReporter::do_build()
{
    auto& manager = require<GlobalBodyManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}

void BodyReporter::init()
{
    InitInfo info;
    do_init(info);
}

void BodyReporter::report_count(BodyCountInfo& info)
{
    do_report_count(info);
    // Record the body count
    m_body_count = info.m_count;
}

void BodyReporter::report_attributes(BodyAttributeInfo& info)
{
    do_report_attributes(info);

    // Record the global body info
    m_body_offset = info.coindices().offset();
    UIPC_ASSERT(m_body_count == info.coindices().size(),
                "Body count mismatch: expected {}, got {}",
                m_body_count,
                info.coindices().size());
    m_body_count = info.coindices().size();
}
}  // namespace uipc::backend::cuda
