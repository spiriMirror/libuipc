#include <affine_body/abd_active_set_reporter.h>

namespace uipc::backend::cuda {
    void ABDActiveSetReporter::Impl::recover_non_penetrate(NonPenetratePositionsInfo &info) {
        abd().body_id_to_q.view().copy_from(non_penetrate_q);
    }

    void ABDActiveSetReporter::Impl::record_non_penetrate() {
        if (non_penetrate_q.size() != abd().body_id_to_q.size())
            non_penetrate_q.resize(abd().body_id_to_q.size());
        non_penetrate_q.view().copy_from(abd().body_id_to_q);
    }

    void ABDActiveSetReporter::Impl::advance_non_penetrate(Float alpha) {
        UIPC_ASSERT(abd().body_id_to_q.size() == non_penetrate_q.size(), "non_penetrate_q's size not matched");
        muda::ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(non_penetrate_q.size(), [
                alpha,
                q = abd().body_id_to_q.cviewer().name("q"),
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
