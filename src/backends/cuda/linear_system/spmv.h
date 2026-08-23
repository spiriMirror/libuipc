#pragma once
#include <type_define.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
// calculate y = a * A * x + b * y
class Spmv
{
  public:
    // symmetric bcoo spmv
    void sym_spmv(Float                                a,
                  cuda_tool::CBCOOMatrixView<Float, 3> A,
                  cuda_tool::CDenseVectorView<Float>   x,
                  Float                                b,
                  cuda_tool::DenseVectorView<Float>    y);

    // reduce by key spmv
    void rbk_spmv(Float                                a,
                  cuda_tool::CBCOOMatrixView<Float, 3> A,
                  cuda_tool::CDenseVectorView<Float>   x,
                  Float                                b,
                  cuda_tool::DenseVectorView<Float>    y);

    // reduce by key symmtric spmv
    void rbk_sym_spmv(Float                                a,
                      cuda_tool::CBCOOMatrixView<Float, 3> A,
                      cuda_tool::CDenseVectorView<Float>   x,
                      Float                                b,
                      cuda_tool::DenseVectorView<Float>    y);

    // reduce by key symmetric spmv with fused dot product
    // computes y = a * A * x  AND  d_dot = x^T * (a * A * x) in a single pass
    // `stream` defaults to the legacy default stream; pass a capture stream
    // when recording a CUDA graph.
    // The triplet count is read on device (`d_triplet_count`) and the grid is
    // sized by `triplet_capacity`, so a captured graph stays valid while the
    // matrix nnz varies within the reserved capacity.
    void rbk_sym_spmv_dot(Float                                a,
                          cuda_tool::CBCOOMatrixView<Float, 3> A,
                          cuda_tool::CDenseVectorView<Float>   x,
                          Float                                b,
                          cuda_tool::DenseVectorView<Float>    y,
                          cuda_tool::VarView<Float>            d_dot,
                          cuda_tool::CDense<IndexT>            d_triplet_count,
                          SizeT                                triplet_capacity,
                          cudaStream_t                         stream = nullptr);

    // debug fallback cpu spmv
    // very slow, only for debug
    void cpu_sym_spmv(Float                                a,
                      cuda_tool::CBCOOMatrixView<Float, 3> A,
                      cuda_tool::CDenseVectorView<Float>   x,
                      Float                                b,
                      cuda_tool::DenseVectorView<Float>    y);
};
}  // namespace uipc::backend::cuda
