#include <cub/warp/warp_reduce.cuh>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda_tool
{
namespace details::fast_segmental_reduce
{
    __host__ __device__ constexpr int b2i(bool b)
    {
        return b ? 1 : 0;
    }
}  // namespace details::fast_segmental_reduce

namespace
{
    // get-key functor over a segment-offset buffer (replaces the device lambda)
    struct fast_segmental_reduce_get_offset_key_op
    {
        CBufferView<int> offset;
        __device__ int operator()(int i) const { return offset(i); }
    };

    // get-value functor over an input buffer (replaces the device lambda)
    template <typename T>
    struct fast_segmental_reduce_get_buffer_value_op
    {
        CBufferView<T> in;
        __device__ T operator()(int i) const { return in(i); }
    };

    // FastSegmentalReduce::reduce (scalar values)
    template <int BlockSize, int WarpSize, typename T, typename FlagsT, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
    __global__ void fast_segmental_reduce_scalar_kernel(BufferView<T> out,
                                                        size_t        in_size,
                                                        GetKeyOp      get_key_op,
                                                        GetValueOp    get_value_op,
                                                        ReduceOp      op)
    {
        using namespace details::fast_segmental_reduce;
        using ValueT = T;
        using Flags  = FlagsT;
        constexpr int warp_size  = WarpSize;
        constexpr int warp_count = BlockSize / WarpSize;

        using WarpReduceInt = cub::WarpReduce<int, warp_size>;
        using WarpReduceT   = cub::WarpReduce<T, warp_size>;

        __shared__ union
        {
            typename WarpReduceInt::TempStorage index_storage[warp_count];
            typename WarpReduceT::TempStorage t_storage[warp_count];
        };

        auto global_thread_id   = blockDim.x * blockIdx.x + threadIdx.x;
        auto thread_id_in_block = threadIdx.x;
        auto warp_id            = thread_id_in_block / warp_size;
        auto lane_id            = thread_id_in_block & (warp_size - 1);

        int    prev_i = -1;
        int    next_i = -1;
        int    i      = -1;
        Flags  flags;
        ValueT value;
        flags.is_cross_warp = 0;

        if(global_thread_id > 0 && global_thread_id < in_size)
        {
            prev_i = get_key_op(global_thread_id - 1);
        }

        if(global_thread_id < in_size - 1)
        {
            next_i = get_key_op(global_thread_id + 1);
        }

        if(global_thread_id < in_size)
        {
            i              = get_key_op(global_thread_id);
            value          = get_value_op(global_thread_id);
            flags.is_valid = 1;
        }
        else
        {
            i                   = -1;
            value               = ValueT{0};
            flags.is_valid      = 0;
            flags.is_cross_warp = 0;
        }

        if(lane_id == 0)
        {
            flags.is_head       = 1;
            flags.is_cross_warp = b2i(prev_i == i);
        }
        else
        {
            flags.is_head = b2i(prev_i != i);

            if(lane_id == warp_size - 1)
            {
                flags.is_cross_warp = b2i(next_i == i);
            }
        }

        flags.flags = WarpReduceInt(index_storage[warp_id])
                          .HeadSegmentedReduce(flags.flags, flags.is_head, op);

        value = WarpReduceT(t_storage[warp_id])
                    .HeadSegmentedReduce(value, flags.is_head, op);


        if(flags.is_head && flags.is_valid)
        {
            if(flags.is_cross_warp)
            {
                auto& out_value = out(i);
                atomic_add(&out_value, value);
            }
            else
            {
               out(i) = value;
            }
        }
    }

    // FastSegmentalReduce::reduce (Eigen::Matrix values)
    template <int BlockSize, int WarpSize, typename T, int M, int N, typename FlagsT, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
    __global__ void fast_segmental_reduce_matrix_kernel(
        BufferView<Eigen::Matrix<T, M, N>> out,
        size_t                             in_size,
        GetKeyOp                           get_key_op,
        GetValueOp                         get_value_op,
        ReduceOp                           op)
    {
        using namespace details::fast_segmental_reduce;
        using Matrix = Eigen::Matrix<T, M, N>;
        using Flags  = FlagsT;
        constexpr int warp_size  = WarpSize;
        constexpr int warp_count = BlockSize / WarpSize;

        using WarpReduceInt = cub::WarpReduce<int, warp_size>;
        using WarpReduceT   = cub::WarpReduce<T, warp_size>;

        __shared__ union
        {
            typename WarpReduceInt::TempStorage index_storage[warp_count];
            typename WarpReduceT::TempStorage t_storage[warp_count];
        };

        auto global_thread_id   = blockDim.x * blockIdx.x + threadIdx.x;
        auto thread_id_in_block = threadIdx.x;
        auto warp_id            = thread_id_in_block / warp_size;
        auto lane_id            = thread_id_in_block & (warp_size - 1);

        int    prev_i = -1;
        int    next_i = -1;
        int    i      = -1;
        Flags  flags;
        Matrix value;
        flags.is_cross_warp = 0;

        if(global_thread_id > 0 && global_thread_id < in_size)
        {
            prev_i = get_key_op(global_thread_id - 1);
        }

        if(global_thread_id < in_size - 1)
        {
            next_i = get_key_op(global_thread_id + 1);
        }

        if(global_thread_id < in_size)
        {
            i              = get_key_op(global_thread_id);
            value          = get_value_op(global_thread_id);
            flags.is_valid = 1;
        }
        else
        {
            i = -1;
            value.setZero();
            flags.is_valid      = 0;
            flags.is_cross_warp = 0;
        }

        if(lane_id == 0)
        {
            flags.is_head       = 1;
            flags.is_cross_warp = b2i(prev_i == i);
        }
        else
        {
            flags.is_head = b2i(prev_i != i);

            if(lane_id == warp_size - 1)
            {
                flags.is_cross_warp = b2i(next_i == i);
            }
        }

        flags.flags = WarpReduceInt(index_storage[warp_id])
                          .HeadSegmentedReduce(flags.flags, flags.is_head, op);

        for(int j = 0; j < M; j++)
        {
            for(int k = 0; k < N; k++)
            {
                value(j, k) =
                    WarpReduceT(t_storage[warp_id])
                        .HeadSegmentedReduce(value(j, k), flags.is_head, op);
            }
        }

        if(flags.is_head && flags.is_valid)
        {
            if(flags.is_cross_warp)
            {
                auto& out_value = out(i);
                eigen::atomic_add(out_value, value);
            }
            else
            {
               out(i) = value;
            }
        }
    }
}  // namespace

template <int BlockSize, int WarpSize>
template <typename T, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    size_t in_size, BufferView<T> out, GetKeyOp get_key_op, GetValueOp get_value_op, ReduceOp op)
{
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");

    using ValueT = T;

    auto          size      = in_size;
    constexpr int block_dim = BlockSize;

    BufferLaunch(this->stream()).fill<ValueT>(out, ValueT{0});

    int block_count = (size + block_dim - 1) / block_dim;
    if(block_count > 0)
        fast_segmental_reduce_scalar_kernel<BlockSize, WarpSize, T, Flags>
            <<<block_count, block_dim, 0, this->stream()>>>(
                out, size, get_key_op, get_value_op, op);

    return *this;
}


template <int BlockSize, int WarpSize>
template <typename T, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    CBufferView<int> offset, CBufferView<T> in, BufferView<T> out, ReduceOp op)
{
    return reduce(
        in.size(),
        out,
        fast_segmental_reduce_get_offset_key_op{offset},
        fast_segmental_reduce_get_buffer_value_op<T>{in},
        op);
}


template <int BlockSize, int WarpSize>
template <typename T, int M, int N, typename GetKeyOp, typename GetValueOp, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    size_t in_size, BufferView<Eigen::Matrix<T, M, N>> out, GetKeyOp get_key_op, GetValueOp get_value_op, ReduceOp op)
{
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");

    using Matrix = Eigen::Matrix<T, M, N>;

    auto          size      = in_size;
    constexpr int block_dim = BlockSize;

    BufferLaunch(this->stream()).fill<Matrix>(out, Matrix::Zero().eval());

    int block_count = (size + block_dim - 1) / block_dim;
    if(block_count > 0)
        fast_segmental_reduce_matrix_kernel<BlockSize, WarpSize, T, M, N, Flags>
            <<<block_count, block_dim, 0, this->stream()>>>(
                out, size, get_key_op, get_value_op, op);

    return *this;
}

template <int BlockSize, int WarpSize>
template <typename T, int M, int N, typename ReduceOp>
FastSegmentalReduce<BlockSize, WarpSize>& FastSegmentalReduce<BlockSize, WarpSize>::reduce(
    CBufferView<int>                    offset,
    CBufferView<Eigen::Matrix<T, M, N>> in,
    BufferView<Eigen::Matrix<T, M, N>>  out,
    ReduceOp                            op)
{
    return reduce(
        in.size(),
        out,
        fast_segmental_reduce_get_offset_key_op{offset},
        fast_segmental_reduce_get_buffer_value_op<Eigen::Matrix<T, M, N>>{in},
        op);
}
}  // namespace uipc::backend::cuda_tool
