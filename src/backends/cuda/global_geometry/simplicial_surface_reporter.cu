#include <global_geometry/simplicial_surface_reporter.h>

namespace uipc::backend::cuda
{
void SimplicialSurfaceReporter::do_build()
{
    auto& global_surf_manager = require<GlobalSimplicialSurfaceManager>();

    BuildInfo info;
    do_build(info);

    global_surf_manager.add_reporter(this);
}

void SimplicialSurfaceReporter::init(SurfaceInitInfo& info)
{
    do_init(info);
}

void SimplicialSurfaceReporter::report_count(GlobalSimplicialSurfaceManager::SurfaceCountInfo& info)
{
    do_report_count(info);
}

void SimplicialSurfaceReporter::report_attributes(GlobalSimplicialSurfaceManager::SurfaceAttributeInfo& info)
{
    do_report_attributes(info);
}
}  // namespace uipc::backend::cuda