#pragma once
#include <cublas_v2.h>
#include <cusparse_v2.h>
#include <muda/type_traits/always.h>
namespace muda
{
template <typename T>
inline constexpr cudaDataType_t cuda_data_type()
{
    if constexpr(std::is_same_v<T, float>)
    {
        return CUDA_R_32F;
    }
    else if constexpr(std::is_same_v<T, double>)
    {
        return CUDA_R_64F;
    }
    else if constexpr(std::is_same_v<T, cuComplex>)
    {
        return CUDA_C_32F;
    }
    else if constexpr(std::is_same_v<T, cuDoubleComplex>)
    {
        return CUDA_C_64F;
    }
    else
    {
        static_assert(always_false_v<T>, "not supported type");
    }
}

constexpr cublasOperation_t cublas_trans_operation(bool b)
{
    return b ? CUBLAS_OP_T : CUBLAS_OP_N;
}

template <typename T>
constexpr cusparseIndexType_t cusparse_index_type()
{
    if constexpr(std::is_same_v<T, int>)
        return cusparseIndexType_t::CUSPARSE_INDEX_32I;
    else if constexpr(std::is_same_v<T, int64_t>)
        return cusparseIndexType_t::CUSPARSE_INDEX_64I;
    else if constexpr(std::is_same_v<T, uint16_t>)
        return cusparseIndexType_t::CUSPARSE_INDEX_16U;
    else
        static_assert(always_false_v<T>, "Unsupported type");
}
}  // namespace muda