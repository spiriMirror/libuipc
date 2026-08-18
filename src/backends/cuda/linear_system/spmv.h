#pragma once
#include <type_define.h>
#include <cuda_tool/muda_compat.h>

namespace uipc::backend::cuda
{
// calculate y = a * A * x + b * y
class Spmv
{
  public:
    // symmetric bcoo spmv
    void sym_spmv(Float                           a,
                  muda::CBCOOMatrixView<Float, 3> A,
                  muda::CDenseVectorView<Float>   x,
                  Float                           b,
                  muda::DenseVectorView<Float>    y);

    // reduce by key spmv
    void rbk_spmv(Float                           a,
                  muda::CBCOOMatrixView<Float, 3> A,
                  muda::CDenseVectorView<Float>   x,
                  Float                           b,
                  muda::DenseVectorView<Float>    y);

    // reduce by key symmtric spmv
    void rbk_sym_spmv(Float                           a,
                      muda::CBCOOMatrixView<Float, 3> A,
                      muda::CDenseVectorView<Float>   x,
                      Float                           b,
                      muda::DenseVectorView<Float>    y);

    // reduce by key symmetric spmv with fused dot product
    // computes y = a * A * x  AND  d_dot = x^T * (a * A * x) in a single pass
    void rbk_sym_spmv_dot(Float                           a,
                          muda::CBCOOMatrixView<Float, 3> A,
                          muda::CDenseVectorView<Float>   x,
                          Float                           b,
                          muda::DenseVectorView<Float>    y,
                          muda::VarView<Float>            d_dot);

    // debug fallback cpu spmv
    // very slow, only for debug
    void cpu_sym_spmv(Float                           a,
                      muda::CBCOOMatrixView<Float, 3> A,
                      muda::CDenseVectorView<Float>   x,
                      Float                           b,
                      muda::DenseVectorView<Float>    y);
};
}  // namespace uipc::backend::cuda
