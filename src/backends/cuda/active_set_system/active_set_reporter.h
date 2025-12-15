#pragma once
#include <sim_system.h>
#include <active_set_system/global_active_set_manager.h>

namespace uipc::backend::cuda {

class ActiveSetReporter : public SimSystem {
public:
    using SimSystem::SimSystem;
    using NonPenetratePositionsInfo = GlobalActiveSetManager::NonPenetratePositionsInfo;

    class BuildInfo {};

protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_report_vertex_offset_count(IndexT &offset, IndexT &count) = 0;
    virtual void do_recover_non_penetrate(NonPenetratePositionsInfo& info) = 0;
private:
    virtual void do_build() override final;

    friend class GlobalActiveSetManager;
    void         report_vertex_offset_count(IndexT &offset, IndexT &count);
    void         recover_non_penetrate(NonPenetratePositionsInfo& info);
};

}
