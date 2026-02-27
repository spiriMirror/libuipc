#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_segmented_reduce.cuh>
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_reduce.html
class DeviceSegmentedReduce : public CubWrapper<DeviceSegmentedReduce>
{
    using Base = CubWrapper<DeviceSegmentedReduce>;

  public:
    using Base::Base;

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT, typename ReductionOp, typename T>
    DeviceSegmentedReduce& Reduce(InputIteratorT       d_in,
                                  OutputIteratorT      d_out,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets,
                                  ReductionOp          reduction_op,
                                  T                    initial_value)
    {


        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedReduce::Reduce(d_temp_storage,
                                                                 temp_storage_bytes,
                                                                 d_in,
                                                                 d_out,
                                                                 num_segments,
                                                                 d_begin_offsets,
                                                                 d_end_offsets,
                                                                 reduction_op,
                                                                 initial_value,
                                                                 _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& Sum(InputIteratorT       d_in,
                               OutputIteratorT      d_out,
                               int                  num_segments,
                               BeginOffsetIteratorT d_begin_offsets,
                               EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedReduce::Sum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& Min(InputIteratorT       d_in,
                               OutputIteratorT      d_out,
                               int                  num_segments,
                               BeginOffsetIteratorT d_begin_offsets,
                               EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedReduce::Min(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& ArgMin(InputIteratorT       d_in,
                                  OutputIteratorT      d_out,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedReduce::ArgMin(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& Max(InputIteratorT       d_in,
                               OutputIteratorT      d_out,
                               int                  num_segments,
                               BeginOffsetIteratorT d_begin_offsets,
                               EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedReduce::Max(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& ArgMax(InputIteratorT       d_in,
                                  OutputIteratorT      d_out,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedReduce::ArgMax(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    // Origin:

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT, typename ReductionOp, typename T>
    DeviceSegmentedReduce& Reduce(void*                d_temp_storage,
                                  size_t&              temp_storage_bytes,
                                  InputIteratorT       d_in,
                                  OutputIteratorT      d_out,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets,
                                  ReductionOp          reduction_op,
                                  T                    initial_value)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedReduce::Reduce(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, reduction_op, initial_value, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& Sum(void*                d_temp_storage,
                               size_t&              temp_storage_bytes,
                               InputIteratorT       d_in,
                               OutputIteratorT      d_out,
                               int                  num_segments,
                               BeginOffsetIteratorT d_begin_offsets,
                               EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedReduce::Sum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& Min(void*                d_temp_storage,
                               size_t&              temp_storage_bytes,
                               InputIteratorT       d_in,
                               OutputIteratorT      d_out,
                               int                  num_segments,
                               BeginOffsetIteratorT d_begin_offsets,
                               EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedReduce::Min(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& ArgMin(void*                d_temp_storage,
                                  size_t&              temp_storage_bytes,
                                  InputIteratorT       d_in,
                                  OutputIteratorT      d_out,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedReduce::ArgMin(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& Max(void*                d_temp_storage,
                               size_t&              temp_storage_bytes,
                               InputIteratorT       d_in,
                               OutputIteratorT      d_out,
                               int                  num_segments,
                               BeginOffsetIteratorT d_begin_offsets,
                               EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedReduce::Max(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceSegmentedReduce& ArgMax(void*           d_temp_storage,
                                  size_t&         temp_storage_bytes,
                                  InputIteratorT  d_in,
                                  OutputIteratorT d_out,
                                  int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedReduce::ArgMax(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedReduce& ArgMax(void*                d_temp_storage,
                                  size_t&              temp_storage_bytes,
                                  InputIteratorT       d_in,
                                  OutputIteratorT      d_out,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedReduce::ArgMax(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"