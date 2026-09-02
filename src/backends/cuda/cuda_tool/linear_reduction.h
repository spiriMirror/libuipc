#pragma once
#include <cuda_tool/cub.h>
#include <cuda_tool/linear_system.h>
#include <cub/block/block_reduce.cuh>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uipc::backend::cuda_tool
{
namespace details
{
    template <typename T>
    struct ScaledSquareSumOp
    {
        __host__ __device__ ScaledSquareSum<T> operator()(const ScaledSquareSum<T>& lhs,
                                                          const ScaledSquareSum<T>& rhs) const
        {
            if(lhs.scale < rhs.scale)
            {
                const T ratio = lhs.scale / rhs.scale;
                return {rhs.scale, rhs.square_sum + lhs.square_sum * ratio * ratio};
            }
            if(rhs.scale < lhs.scale)
            {
                const T ratio = rhs.scale / lhs.scale;
                return {lhs.scale, lhs.square_sum + rhs.square_sum * ratio * ratio};
            }
            if(lhs.scale == rhs.scale)
                return {lhs.scale, lhs.square_sum + rhs.square_sum};

            // At least one scale is NaN. Arithmetic propagation preserves the
            // diagnostic behavior expected by the PCG finite-value checks.
            return {lhs.scale + rhs.scale, lhs.square_sum + rhs.square_sum};
        }
    };

    template <typename T>
    __host__ __device__ ScaledSquareSum<T> make_scaled_square_sum(T value)
    {
        const T magnitude = value < T{0} ? -value : value;
        return magnitude == T{0} ? ScaledSquareSum<T>{} :
                                   ScaledSquareSum<T>{magnitude, T{1}};
    }

    inline constexpr int linear_reduction_block_size = 256;

    template <typename T>
    __global__ void linear_dot_partial_kernel(CBufferView<T> x,
                                              CBufferView<T> y,
                                              BufferView<T>  partials)
    {
        using BlockReduce = cub::BlockReduce<T, linear_reduction_block_size>;
        __shared__ typename BlockReduce::TempStorage temp_storage;

        const size_t i =
            static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
        const T value = i < x.size() ? x[i] * y[i] : T{0};
        const T sum   = BlockReduce(temp_storage).Sum(value);
        if(threadIdx.x == 0)
            partials[blockIdx.x] = sum;
    }

    template <typename T>
    __global__ void linear_norm_partial_kernel(CBufferView<T> x,
                                               BufferView<ScaledSquareSum<T>> partials)
    {
        using State = ScaledSquareSum<T>;
        using BlockReduce = cub::BlockReduce<State, linear_reduction_block_size>;
        __shared__ typename BlockReduce::TempStorage temp_storage;

        const size_t i =
            static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
        const State value = i < x.size() ? make_scaled_square_sum(x[i]) : State{};
        const State sum =
            BlockReduce(temp_storage).Reduce(value, ScaledSquareSumOp<T>{});
        if(threadIdx.x == 0)
            partials[blockIdx.x] = sum;
    }

    inline int linear_reduction_block_count(size_t value_count)
    {
        if(value_count > static_cast<size_t>(std::numeric_limits<int>::max()))
            throw std::length_error{"LinearSystemContext reduction exceeds INT_MAX"};
        return static_cast<int>((value_count + linear_reduction_block_size - 1)
                                / linear_reduction_block_size);
    }
}  // namespace details

template <typename T>
inline T LinearSystemContext::dot_view(CBufferView<T> x, CBufferView<T> y)
{
    if(x.size() != y.size())
        throw std::invalid_argument{"LinearSystemContext::dot size mismatch"};
    if(x.size() == 0)
        return T{0};

    auto& storage     = reduction_storage<T>();
    int   block_count = details::linear_reduction_block_count(x.size());
    storage.dot_partials.resize_discard(block_count, m_stream);

    details::linear_dot_partial_kernel<T>
        <<<block_count, details::linear_reduction_block_size, 0, m_stream>>>(
            x, y, storage.dot_partials.view());
    CUDA_TOOL_CHECK(cudaGetLastError());
    DeviceReduce(m_stream).Sum(storage.dot_partials.data(), storage.dot_result.data(), block_count);

    T result{};
    storage.dot_result.copy_to(result, m_stream);
    CUDA_TOOL_CHECK(cudaStreamSynchronize(m_stream));
    return result;
}

template <typename T>
inline T LinearSystemContext::norm_view(CBufferView<T> x)
{
    if(x.size() == 0)
        return T{0};

    using State       = details::ScaledSquareSum<T>;
    auto& storage     = reduction_storage<T>();
    int   block_count = details::linear_reduction_block_count(x.size());
    storage.norm_partials.resize_discard(block_count, m_stream);

    details::linear_norm_partial_kernel<T>
        <<<block_count, details::linear_reduction_block_size, 0, m_stream>>>(
            x, storage.norm_partials.view());
    CUDA_TOOL_CHECK(cudaGetLastError());
    DeviceReduce(m_stream).Reduce(storage.norm_partials.data(),
                                  storage.norm_result.data(),
                                  block_count,
                                  details::ScaledSquareSumOp<T>{},
                                  State{});

    State result{};
    storage.norm_result.copy_to(result, m_stream);
    CUDA_TOOL_CHECK(cudaStreamSynchronize(m_stream));
    return result.scale * std::sqrt(result.square_sum);
}
}  // namespace uipc::backend::cuda_tool
