#pragma once
#include <linear_system/iterative_solver.h>
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/graph.h>
#include <array>

namespace uipc::backend::cuda
{
// Fused PCG: keeps dot-product scalars (rz, pAp, rz_new) on device
// to eliminate per-iteration host synchronizations.  The update kernels read
// alpha = rz/pAp and beta = rz_new/rz directly from device memory.
// SpMV and dot(p,Ap) are fused into a single kernel pass.
// Convergence is checked every `check_interval` iterations via a single D2H copy.
//
// CUDA graph replay: the per-iteration kernel chain (spmv_dot -> update_xr ->
// preconditioner -> dot -> converged -> update_p -> swap_rz) launches ~10
// tiny kernels per iteration whose launch gaps dominate the wall time
// (~80us of ~118us per iteration on case2-scale scenes). When
// `linear_system/use_cuda_graph` is on (default), a block of
// `check_interval` iterations is recorded once per (buffer-set, N) and
// replayed as one graph launch; kernels, arguments and ordering are
// identical to the non-graph path, so numerics are unchanged. If capture
// fails (e.g. a preconditioner launches outside the capture stream), the
// solver permanently falls back to the plain loop for that instance.
class LinearFusedPCG : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) override;

  private:
    using DeviceDenseVector = cuda_tool::DeviceDenseVector<Float>;

    SizeT fused_pcg(cuda_tool::DenseVectorView<Float>  x,
                    cuda_tool::CDenseVectorView<Float> b,
                    SizeT                              max_iter);
    void  check_init_rz_nan_inf(Float rz);
    void  check_iter_rz_nan_inf(Float rz, SizeT k);

    // One iteration of the PCG loop body on `stream` (the unit of graph
    // capture and of the uncaptured fallback path).
    void run_iteration(cuda_tool::DenseVectorView<Float> x, cudaStream_t stream, bool timed);

    // Capture `interval` iterations into m_graph (no execution during
    // capture); on any failure disable graph replay for this instance.
    void rebuild_graph(cuda_tool::DenseVectorView<Float>  x,
                       cuda_tool::CDenseVectorView<Float> b,
                       SizeT                              interval,
                       SizeT                              max_iter);
    bool  graph_key_matches(cuda_tool::DenseVectorView<Float>  x,
                            cuda_tool::CDenseVectorView<Float> b,
                            SizeT                              interval,
                            SizeT                              max_iter) const;
    void  destroy_graph();

    DeviceDenseVector r;
    DeviceDenseVector z;
    DeviceDenseVector p;
    DeviceDenseVector Ap;

    cuda_tool::DeviceVar<Float>  d_rz;
    cuda_tool::DeviceVar<Float>  d_pAp;
    cuda_tool::DeviceVar<Float>  d_rz_new;
    cuda_tool::DeviceVar<IndexT> d_converged;
    // rz_tol on device so a captured graph survives rz_tol changes
    cuda_tool::DeviceVar<Float>  d_rz_tol;

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;
    SizeT check_interval  = 5;

    // --- CUDA graph state ---
    IndexT m_use_cuda_graph = 1;  // config: linear_system/use_cuda_graph
    cuda_tool::GraphCapture m_graph;
    // validity key: every device pointer baked into the captured kernels
    std::array<const void*, 12> m_graph_ptrs{};
    SizeT m_graph_n        = 0;
    SizeT m_graph_interval = 0;
    SizeT m_graph_max_iter = 0;
    SizeT m_graph_triplets = 0;  // baked into the captured SpMV kernel args
};
}  // namespace uipc::backend::cuda
