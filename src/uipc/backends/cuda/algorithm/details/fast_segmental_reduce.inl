#include <cub/warp/warp_reduce.cuh>
#include <muda/ext/eigen/atomic.h>
namespace muda
{
//constexpr int BlockSize = 128;
//constexpr int WarpSize  = 32;
//using T                 = float;
//constexpr int M         = 3;
//constexpr int N         = 3;

namespace details::fast_segmental_reduce
{
    __host__ __device__ constexpr int b2i(bool b)
    {
        return b ? 1 : 0;
    }
}  // namespace details::fast_segmental_reduce

template <int BlockSize, int WarpSize>
template <typename T, int M, int N, typename ReduceOp>
void FastSegmentalReduce<BlockSize, WarpSize>::reduce(CBufferView<int> offset,
                                                      CBufferView<Eigen::Matrix<T, M, N>> in,
                                                      BufferView<Eigen::Matrix<T, M, N>> out,
                                                      ReduceOp op)
{
    using namespace details::fast_segmental_reduce;
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");

    using Matrix = Eigen::Matrix<T, M, N>;

    auto                   size       = in.size();
    constexpr int          warp_size  = WarpSize;
    constexpr unsigned int warp_mask  = ~0u;
    constexpr int          block_dim  = BlockSize;
    constexpr int          warp_count = block_dim / warp_size;

    BufferLaunch(this->stream()).fill<Matrix>(out, Matrix::Zero().eval());

    int block_count = (size + block_dim - 1) / block_dim;
    Launch(block_count, block_dim)
        .kernel_name("segmental_reduce")
        .apply(
            [in     = in.cviewer().name("in"),
             out    = out.viewer().name("out"),
             offset = offset.cviewer().name("offset"),
             op] __device__() mutable
            {
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

                if(global_thread_id > 0 && global_thread_id < in.total_size())
                {
                    prev_i = offset(global_thread_id - 1);
                }

                if(global_thread_id < in.total_size() - 1)
                {
                    next_i = offset(global_thread_id + 1);
                }

                if(global_thread_id < in.total_size())
                {
                    i              = offset(global_thread_id);
                    value          = in(global_thread_id);
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
            });
}
template <int BlockSize, int WarpSize>
template <typename T, typename ReduceOp>
void FastSegmentalReduce<BlockSize, WarpSize>::reduce(CBufferView<int> offset,
                                                      CBufferView<T>   in,
                                                      BufferView<T>    out,
                                                      ReduceOp         op)
{
    using namespace details::fast_segmental_reduce;
    static_assert(std::is_floating_point_v<T> || std::is_integral_v<T>,
                  "FastSegmentalReduce only supports floating point and integral types");

    using ValueT = T;

    auto                   size       = in.size();
    constexpr int          warp_size  = WarpSize;
    constexpr unsigned int warp_mask  = ~0u;
    constexpr int          block_dim  = BlockSize;
    constexpr int          warp_count = block_dim / warp_size;

    BufferLaunch(this->stream()).fill<ValueT>(out, ValueT{0});

    int block_count = (size + block_dim - 1) / block_dim;
    Launch(block_count, block_dim)
        .kernel_name("segmental_reduce")
        .apply(
            [in     = in.cviewer().name("in"),
             out    = out.viewer().name("out"),
             offset = offset.cviewer().name("offset"),
             op     = op] __device__() mutable
            {
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

                if(global_thread_id > 0 && global_thread_id < in.total_size())
                {
                    prev_i = offset(global_thread_id - 1);
                }

                if(global_thread_id < in.total_size() - 1)
                {
                    next_i = offset(global_thread_id + 1);
                }

                if(global_thread_id < in.total_size())
                {
                    value          = in(global_thread_id);
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
            });
}
}  // namespace muda
