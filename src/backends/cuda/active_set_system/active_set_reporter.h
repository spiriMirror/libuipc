#pragma once
#include <sim_system.h>
#include <active_set_system/global_active_set_manager.h>

namespace uipc::backend::cuda {

class ActiveSetReporter : public SimSystem {
public:
    using SimSystem::SimSystem;
    using NonPenetratePositionsInfo = GlobalActiveSetManager::NonPenetratePositionInfo;

    class BuildInfo {};

protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_report_vertex_offset_count(IndexT &offset, IndexT &count) = 0;
    virtual void do_recover_non_penetrate(NonPenetratePositionsInfo& info) = 0;
    virtual void do_record_non_penetrate_state() = 0;
    virtual void do_advance_non_penetrate_state(Float alpha) = 0;
private:
    virtual void do_build() override final;

    friend class GlobalActiveSetManager;
    void         report_vertex_offset_count(IndexT &offset, IndexT &count);
    void         record_non_penetrate_state();
    void         advance_non_penetrate_state(Float alpha);
    void         recover_non_penetrate(NonPenetratePositionsInfo& info);
};

}
