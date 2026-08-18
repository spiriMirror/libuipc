#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_reduce.cuh>
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_reduce.html
class DeviceReduce : public CubWrapper<DeviceReduce>
{
    using Base = CubWrapper<DeviceReduce>;

  public:
    using Base::Base;

    // DeviceVector:

    template <typename InputIteratorT, typename OutputIteratorT, typename ReductionOpT, typename T>
    DeviceReduce& Reduce(InputIteratorT  d_in,
                         OutputIteratorT d_out,
                         int             num_items,
                         ReductionOpT    reduction_op,
                         T               init)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceReduce::Reduce(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, reduction_op, init, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& Sum(InputIteratorT d_in, OutputIteratorT d_out, int num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceReduce::Sum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& Min(InputIteratorT d_in, OutputIteratorT d_out, int num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceReduce::Min(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& ArgMin(InputIteratorT d_in, OutputIteratorT d_out, int num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceReduce::ArgMin(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& Max(InputIteratorT d_in, OutputIteratorT d_out, int num_items)
    {

        MUDA_CUB_WRAPPER_IMPL(cub::DeviceReduce::Max(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& ArgMax(InputIteratorT d_in, OutputIteratorT d_out, int num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceReduce::ArgMax(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename KeysInputIteratorT, typename UniqueOutputIteratorT, typename ValuesInputIteratorT, typename AggregatesOutputIteratorT, typename NumRunsOutputIteratorT, typename ReductionOpT>
    DeviceReduce& ReduceByKey(KeysInputIteratorT        d_keys_in,
                              UniqueOutputIteratorT     d_unique_out,
                              ValuesInputIteratorT      d_values_in,
                              AggregatesOutputIteratorT d_aggregates_out,
                              NumRunsOutputIteratorT    d_num_runs_out,
                              ReductionOpT              reduction_op,
                              int                       num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceReduce::ReduceByKey(d_temp_storage,
                                                             temp_storage_bytes,
                                                             d_keys_in,
                                                             d_unique_out,
                                                             d_values_in,
                                                             d_aggregates_out,
                                                             d_num_runs_out,
                                                             reduction_op,
                                                             num_items));
    }


    // Origin:

    template <typename InputIteratorT, typename OutputIteratorT, typename ReductionOpT, typename T>
    DeviceReduce& Reduce(void*           d_temp_storage,
                         size_t&         temp_storage_bytes,
                         InputIteratorT  d_in,
                         OutputIteratorT d_out,
                         int             num_items,
                         ReductionOpT    reduction_op,
                         T               init)
    {

        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceReduce::Reduce(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, reduction_op, init, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& Sum(void*           d_temp_storage,
                      size_t&         temp_storage_bytes,
                      InputIteratorT  d_in,
                      OutputIteratorT d_out,
                      int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceReduce::Sum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& Min(void*           d_temp_storage,
                      size_t&         temp_storage_bytes,
                      InputIteratorT  d_in,
                      OutputIteratorT d_out,
                      int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceReduce::Min(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& ArgMin(void*           d_temp_storage,
                         size_t&         temp_storage_bytes,
                         InputIteratorT  d_in,
                         OutputIteratorT d_out,
                         int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceReduce::ArgMin(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& Max(void*           d_temp_storage,
                      size_t&         temp_storage_bytes,
                      InputIteratorT  d_in,
                      OutputIteratorT d_out,
                      int             num_items)
    {

        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceReduce::Max(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceReduce& ArgMax(void*           d_temp_storage,
                         size_t&         temp_storage_bytes,
                         InputIteratorT  d_in,
                         OutputIteratorT d_out,
                         int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceReduce::ArgMax(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename KeysInputIteratorT, typename UniqueOutputIteratorT, typename ValuesInputIteratorT, typename AggregatesOutputIteratorT, typename NumRunsOutputIteratorT, typename ReductionOpT>
    DeviceReduce& ReduceByKey(void*                     d_temp_storage,
                              size_t&                   temp_storage_bytes,
                              KeysInputIteratorT        d_keys_in,
                              UniqueOutputIteratorT     d_unique_out,
                              ValuesInputIteratorT      d_values_in,
                              AggregatesOutputIteratorT d_aggregates_out,
                              NumRunsOutputIteratorT    d_num_runs_out,
                              ReductionOpT              reduction_op,
                              int                       num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceReduce::ReduceByKey(d_temp_storage,
                                           temp_storage_bytes,
                                           d_keys_in,
                                           d_unique_out,
                                           d_values_in,
                                           d_aggregates_out,
                                           d_num_runs_out,
                                           reduction_op,
                                           num_items));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"