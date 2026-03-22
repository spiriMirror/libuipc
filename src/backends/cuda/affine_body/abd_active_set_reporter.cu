#include <affine_body/abd_active_set_reporter.h>

namespace uipc::backend::cuda {
    void ABDActiveSetReporter::Impl::recover_non_penetrate(NonPenetratePositionsInfo &info) {
        affine_body_dynamics->overwrite_qs(non_penetrate_q.view());
    }

    void ABDActiveSetReporter::Impl::record_non_penetrate() {
        auto qs = affine_body_dynamics->qs();
        if(non_penetrate_q.size() != qs.size())
            non_penetrate_q.resize(qs.size());
        non_penetrate_q.view().copy_from(qs);
    }

    void ABDActiveSetReporter::Impl::advance_non_penetrate(Float alpha) {
        auto qs = affine_body_dynamics->qs();
        UIPC_ASSERT(qs.size() == non_penetrate_q.size(), "non_penetrate_q's size not matched");
        muda::ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(non_penetrate_q.size(), [
                alpha,
                q = qs.cviewer().name("q"),
                non_penetrate_q = non_penetrate_q.viewer().name("non_penetrate_q")
            ] __device__ (int i) mutable {
                non_penetrate_q(i) += alpha * (q(i) - non_penetrate_q(i));
            });
    }

    void ABDActiveSetReporter::do_build(BuildInfo &info) {
        m_impl.affine_body_dynamics = require<AffineBodyDynamics>();
        m_impl.vertex_reporter = require<AffineBodyVertexReporter>();
    }

    void ABDActiveSetReporter::do_report_vertex_offset_count(IndexT &offset, IndexT &count) {
        offset = m_impl.vertex_reporter->vertex_offset();
        count = m_impl.vertex_reporter->vertex_count();
    }

    void ABDActiveSetReporter::do_recover_non_penetrate(NonPenetratePositionsInfo &info) {
        m_impl.recover_non_penetrate(info);
    }

    void ABDActiveSetReporter::do_record_non_penetrate_state() {
        m_impl.record_non_penetrate();
    }

    void ABDActiveSetReporter::do_advance_non_penetrate_state(Float alpha) {
        m_impl.advance_non_penetrate(alpha);
    }

REGISTER_SIM_SYSTEM(ABDActiveSetReporter);
}
