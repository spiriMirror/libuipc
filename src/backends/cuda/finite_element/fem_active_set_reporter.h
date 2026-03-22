#pragma once
#include <active_set_system/active_set_reporter.h>
#include <finite_element/finite_element_vertex_reporter.h>

namespace uipc::backend::cuda {

class FEMActiveSetReporter : public ActiveSetReporter {
public:
    using ActiveSetReporter::ActiveSetReporter;

    class Impl {
    public:
        SimSystemSlot<FiniteElementVertexReporter> vertex_reporter;
        SimSystemSlot<FiniteElementMethod>         finite_element_method;

        void recover_non_penetrate(NonPenetratePositionsInfo &info);
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
