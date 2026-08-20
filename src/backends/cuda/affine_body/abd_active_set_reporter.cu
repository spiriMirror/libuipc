#include <affine_body/abd_active_set_reporter.h>

namespace uipc::backend::cuda {
namespace
{
    __global__ void abd_active_set_reporter_advance_non_penetrate_kernel(
        Float                           alpha,
        cuda_tool::CBufferView<Vector12> q,
        cuda_tool::BufferView<Vector12>  non_penetrate_q,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        non_penetrate_q(i) += alpha * (q(i) - non_penetrate_q(i));
    }
}  // namespace

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
        int n = (int)non_penetrate_q.size();
        if(n > 0)
        {
            auto k = abd_active_set_reporter_advance_non_penetrate_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                alpha, qs.cview(), non_penetrate_q.view(), n);
        }
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
