#include <global_geometry/surface_reporter.h>

namespace uipc::backend::cuda
{
void SurfaceReporter::report_count(GlobalSimpicialSurfaceManager::SurfaceCountInfo& info)
{
    do_report_count(info);
}
void SurfaceReporter::report_attributes(GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info)
{
    do_report_attributes(info);
}
}  // namespace uipc::backend::cuda