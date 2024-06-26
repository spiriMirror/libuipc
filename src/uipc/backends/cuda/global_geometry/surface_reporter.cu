#include <global_geometry/surface_reporter.h>

namespace uipc::backend::cuda
{
void SurfaceReporter::report_count(GlobalSurfaceManager::SurfaceCountInfo& info)
{
    do_report_count(info);
}
void SurfaceReporter::report_attributes(GlobalSurfaceManager::SurfaceAttributeInfo& info)
{
    do_report_attributes(info);
}
}  // namespace uipc::backend::cuda