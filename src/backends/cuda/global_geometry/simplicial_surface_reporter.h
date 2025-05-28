#pragma once
#include <sim_system.h>
#include <global_geometry/global_simplicial_surface_manager.h>

namespace uipc::backend::cuda
{
class SimplicialSurfaceReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    class BuildInfo
    {
      public:
    };

    using SurfaceInitInfo  = GlobalSimpicialSurfaceManager::SurfaceInitInfo;
    using SurfaceCountInfo = GlobalSimpicialSurfaceManager::SurfaceCountInfo;
    using SurfaceAttributeInfo = GlobalSimpicialSurfaceManager::SurfaceAttributeInfo;

  protected:
    virtual void do_build(BuildInfo& info)                        = 0;
    virtual void do_init(SurfaceInitInfo& info)                   = 0;
    virtual void do_report_count(SurfaceCountInfo& info)          = 0;
    virtual void do_report_attributes(SurfaceAttributeInfo& info) = 0;

  private:
    friend class GlobalSimpicialSurfaceManager;
    virtual void do_build() final override;

    void init(SurfaceInitInfo& info);
    void report_count(SurfaceCountInfo& info);
    void report_attributes(SurfaceAttributeInfo& info);

    friend class GlobalSimpicialSurfaceManager;
    SizeT m_index = ~0ull;
};
}  // namespace uipc::backend::cuda