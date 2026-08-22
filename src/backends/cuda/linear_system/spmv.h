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
    void rbk_sym_spmv_dot(Float                                a,
                          cuda_tool::CBCOOMatrixView<Float, 3> A,
                          cuda_tool::CDenseVectorView<Float>   x,
                          Float                                b,
                          cuda_tool::DenseVectorView<Float>    y,
                          cuda_tool::VarView<Float>            d_dot);

    // debug fallback cpu spmv
    // very slow, only for debug
    void cpu_sym_spmv(Float                                a,
                      cuda_tool::CBCOOMatrixView<Float, 3> A,
                      cuda_tool::CDenseVectorView<Float>   x,
                      Float                                b,
                      cuda_tool::DenseVectorView<Float>    y);
};
}  // namespace uipc::backend::cuda
