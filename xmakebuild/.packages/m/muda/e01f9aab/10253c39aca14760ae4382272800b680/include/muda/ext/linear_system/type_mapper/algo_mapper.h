#pragma once
#include <cublas_v2.h>
#include <cusparse_v2.h>
#include <muda/type_traits/always.h>
namespace muda
{
class LinearSystemAlgorithm
{
  public:
    // convert for compatibility
    constexpr static cusparseSpMVAlg_t SPMV_ALG_DEFAULT = (cusparseSpMVAlg_t)0;
};
}  // namespace muda