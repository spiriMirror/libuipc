#include <global_geometry/simplicial_surface_reporter.h>

namespace uipc::backend::cuda
{
void SimplicialSurfaceReporter::report_count(GlobalSimpicialSurfaceManager::SurfaceCountInfo& info)
{
    do_report_count(info);
}
void SimplicialSurfaceReporter::report_attributes(GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info)
{
    do_report_attributes(info);
}
}  // namespace uipc::backend::cuda