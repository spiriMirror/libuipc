#pragma once
#include <sim_system.h>
#include <global_surface_manager.h>

namespace uipc::backend::cuda
{
class SurfaceReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

  protected:
    virtual void do_report_count(GlobalSurfaceManager::SurfaceCountInfo& info) = 0;
    virtual void do_report_attributes(GlobalSurfaceManager::SurfaceAttributeInfo& info) = 0;

  private:
    friend class GlobalSurfaceManager;
    void report_count(GlobalSurfaceManager::SurfaceCountInfo& info);
    void report_attributes(GlobalSurfaceManager::SurfaceAttributeInfo& info);

    friend class GlobalSurfaceManager;
    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda