#pragma once
#include <active_set_system/active_set_reporter.h>
#include <affine_body/affine_body_vertex_reporter.h>

namespace uipc::backend::cuda {

class ABDActiveSetReporter : public ActiveSetReporter {
public:
    using ActiveSetReporter::ActiveSetReporter;

    class Impl {
    public:
        SimSystemSlot<AffineBodyDynamics>               affine_body_dynamics;
        SimSystemSlot<AffineBodyVertexReporter>         vertex_reporter;
        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }

        muda::DeviceBuffer<Vector12> non_penetrate_q;

        void recover_non_penetrate(NonPenetratePositionsInfo &info);
        void record_non_penetrate();
        void advance_non_penetrate(Float alpha);
    };

protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_report_vertex_offset_count(IndexT &offset, IndexT &count) override;
    virtual void do_recover_non_penetrate(NonPenetratePositionsInfo &info) override;
    virtual void do_record_non_penetrate_state() override;
    virtual void do_advance_non_penetrate_state(Float alpha) override;

private:
    Impl m_impl;
};

}
