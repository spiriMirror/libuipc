#pragma once
#include <muda/cub/device/cub_wrapper.h>
#if CUB_VERSION >= 200200
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_adjacent_difference.cuh>
#else
namespace cub
{
class Difference;
}
#endif

namespace muda
{
#if CUDA_VERSION < 13000
#define MUDA_DIFFERENCE cub::Difference
#else
#define MUDA_DIFFERENCE ::cuda::std::minus<>
#endif

//ref: https://nvlabs.github.io/cub/structcub_1_1_device_adjacent_difference.html
class DeviceAdjacentDifference : public CubWrapper<DeviceAdjacentDifference>
{
    using Base = CubWrapper<DeviceAdjacentDifference>;

  public:
    using Base::Base;
    // DeviceVector:

    template <typename InputIteratorT, typename OutputIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractLeftCopy(InputIteratorT  d_in,
                                               OutputIteratorT d_out,
                                               int             num_items,
                                               DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceAdjacentDifference::SubtractLeftCopy(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, difference_op, _stream));
    }

    template <typename RandomAccessIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractLeft(RandomAccessIteratorT d_in,
                                           int                   num_items,
                                           DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceAdjacentDifference::SubtractLeft(
            d_temp_storage, temp_storage_bytes, d_in, num_items, difference_op, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractRightCopy(InputIteratorT  d_in,
                                                OutputIteratorT d_out,
                                                int             num_items,
                                                DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceAdjacentDifference::SubtractRightCopy(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, difference_op, _stream));
    }

    template <typename RandomAccessIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractRight(RandomAccessIteratorT d_in,
                                            int                   num_items,
                                            DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceAdjacentDifference::SubtractRight(
            d_temp_storage, temp_storage_bytes, d_in, num_items, difference_op, _stream));
    }

    // Origin:

    template <typename InputIteratorT, typename OutputIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractLeftCopy(void*   d_temp_storage,
                                               size_t& temp_storage_bytes,
                                               InputIteratorT  d_in,
                                               OutputIteratorT d_out,
                                               int             num_items,
                                               DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceAdjacentDifference::SubtractLeftCopy(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, difference_op, _stream));
    }

    template <typename RandomAccessIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractLeft(void*   d_temp_storage,
                                           size_t& temp_storage_bytes,
                                           RandomAccessIteratorT d_in,
                                           int                   num_items,
                                           DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceAdjacentDifference::SubtractLeft(
            d_temp_storage, temp_storage_bytes, d_in, num_items, difference_op, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractRightCopy(void*   d_temp_storage,
                                                size_t& temp_storage_bytes,
                                                InputIteratorT  d_in,
                                                OutputIteratorT d_out,
                                                int             num_items,
                                                DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceAdjacentDifference::SubtractRightCopy(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, difference_op, _stream));
    }

    template <typename RandomAccessIteratorT, typename DifferenceOpT = MUDA_DIFFERENCE>
    DeviceAdjacentDifference& SubtractRight(void*   d_temp_storage,
                                            size_t& temp_storage_bytes,
                                            RandomAccessIteratorT d_in,
                                            int                   num_items,
                                            DifferenceOpT difference_op = {})
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceAdjacentDifference::SubtractRight(
            d_temp_storage, temp_storage_bytes, d_in, num_items, difference_op, _stream));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"
#endif
