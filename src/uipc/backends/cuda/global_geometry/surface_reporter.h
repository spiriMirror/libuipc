#pragma once
#include <sim_system.h>
#include <global_geometry/global_surface_manager.h>

namespace uipc::backend::cuda
{
class SurfaceReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_report_count(GlobalSimpicialSurfaceManager::SurfaceCountInfo& info) = 0;
    virtual void do_report_attributes(GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info) = 0;

  private:
    friend class GlobalSimpicialSurfaceManager;
    void report_count(GlobalSimpicialSurfaceManager::SurfaceCountInfo& info);
    void report_attributes(GlobalSimpicialSurfaceManager::SurfaceAttributeInfo& info);

    friend class GlobalSimpicialSurfaceManager;
    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda