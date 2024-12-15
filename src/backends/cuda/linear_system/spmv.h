#pragma once
#include <type_define.h>
#include <muda/buffer/device_buffer.h>
#include <muda/ext/linear_system/bcoo_matrix_view.h>
#include <muda/ext/linear_system/bsr_matrix_view.h>
#include <muda/ext/linear_system/dense_vector_view.h>
#include <muda/ext/linear_system/device_dense_vector.h>
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
};
}  // namespace uipc::backend::cuda
