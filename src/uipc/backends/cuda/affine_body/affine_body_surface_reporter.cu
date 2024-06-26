#include <affine_body/affine_body_surface_reporter.h>

namespace uipc::backend::cuda
{
class SimSystemCreator<AffinebodySurfaceReporter>
{
  public:
    static U<AffinebodySurfaceReporter> create(SimEngine& engine)
    {
        return has_affine_body_constitution(engine) ?
                   make_unique<AffinebodySurfaceReporter>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(AffinebodySurfaceReporter);


void AffinebodySurfaceReporter::do_build()
{
    m_impl.affine_body_dynamics        = find<AffineBodyDynamics>();
    m_impl.affine_body_vertex_reporter = find<AffineBodyVertexReporter>();
}

void AffinebodySurfaceReporter::do_report_count(GlobalSurfaceManager::SurfaceCountInfo& info)
{
}

void AffinebodySurfaceReporter::do_report_attributes(GlobalSurfaceManager::SurfaceAttributeInfo& info)
{
}
void AffinebodySurfaceReporter::Impl::init_surface_on_host(backend::WorldVisitor& w)
{
    auto geo_slots = w.scene().geometries();

    for(auto&& body_info : abd().h_body_infos)
    {
        body_info.geometry_slot_index();
    }
}
}  // namespace uipc::backend::cuda
