#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_select.cuh>
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_select.html
class DeviceSelect : public CubWrapper<DeviceSelect>
{
    using Base = CubWrapper<DeviceSelect>;

  public:
    using Base::Base;

    template <typename InputIteratorT, typename FlagIterator, typename OutputIteratorT, typename NumSelectedIteratorT>
    DeviceSelect& Flagged(InputIteratorT       d_in,
                          FlagIterator         d_flags,
                          OutputIteratorT      d_out,
                          NumSelectedIteratorT d_num_selected_out,
                          int                  num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSelect::Flagged(
            d_temp_storage, temp_storage_bytes, d_in, d_flags, d_out, d_num_selected_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename NumSelectedIteratorT, typename SelectOp>
    DeviceSelect& If(InputIteratorT       d_in,
                     OutputIteratorT      d_out,
                     NumSelectedIteratorT d_num_selected_out,
                     int                  num_items,
                     SelectOp             select_op)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSelect::If(
            d_temp_storage, temp_storage_bytes, d_in, d_out, d_num_selected_out, num_items, select_op, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename NumSelectedIteratorT>
    DeviceSelect& Unique(InputIteratorT       d_in,
                         OutputIteratorT      d_out,
                         NumSelectedIteratorT d_num_selected_out,
                         int                  num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSelect::Unique(
            d_temp_storage, temp_storage_bytes, d_in, d_out, d_num_selected_out, num_items, _stream));
    }
#if CUB_VERSION >= 200200
    template <typename KeyInputIteratorT, typename ValueInputIteratorT, typename KeyOutputIteratorT, typename ValueOutputIteratorT, typename NumSelectedIteratorT>
    DeviceSelect& UniqueByKey(KeyInputIteratorT    d_keys_in,
                              ValueInputIteratorT  d_values_in,
                              KeyOutputIteratorT   d_keys_out,
                              ValueOutputIteratorT d_values_out,
                              NumSelectedIteratorT d_num_selected_out,
                              int                  num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSelect::UniqueByKey(d_temp_storage,
                                                             temp_storage_bytes,
                                                             d_keys_in,
                                                             d_values_in,
                                                             d_keys_out,
                                                             d_values_out,
                                                             d_num_selected_out,
                                                             num_items,
                                                             _stream));
    }
#endif

    // Origin:

    template <typename InputIteratorT, typename FlagIterator, typename OutputIteratorT, typename NumSelectedIteratorT>
    DeviceSelect& Flagged(void*                d_temp_storage,
                          size_t&              temp_storage_bytes,
                          InputIteratorT       d_in,
                          FlagIterator         d_flags,
                          OutputIteratorT      d_out,
                          NumSelectedIteratorT d_num_selected_out,
                          int                  num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSelect::Flagged(
            d_temp_storage, temp_storage_bytes, d_in, d_flags, d_out, d_num_selected_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename NumSelectedIteratorT, typename SelectOp>
    DeviceSelect& If(void*                d_temp_storage,
                     size_t&              temp_storage_bytes,
                     InputIteratorT       d_in,
                     OutputIteratorT      d_out,
                     NumSelectedIteratorT d_num_selected_out,
                     int                  num_items,
                     SelectOp             select_op)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSelect::If(
            d_temp_storage, temp_storage_bytes, d_in, d_out, d_num_selected_out, num_items, select_op, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename NumSelectedIteratorT>
    DeviceSelect& Unique(void*                d_temp_storage,
                         size_t&              temp_storage_bytes,
                         InputIteratorT       d_in,
                         OutputIteratorT      d_out,
                         NumSelectedIteratorT d_num_selected_out,
                         int                  num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSelect::Unique(
            d_temp_storage, temp_storage_bytes, d_in, d_out, d_num_selected_out, num_items, _stream));
    }
#if CUB_VERSION >= 200200
    template <typename KeyInputIteratorT, typename ValueInputIteratorT, typename KeyOutputIteratorT, typename ValueOutputIteratorT, typename NumSelectedIteratorT>
    DeviceSelect& UniqueByKey(void*                d_temp_storage,
                              size_t&              temp_storage_bytes,
                              KeyInputIteratorT    d_keys_in,
                              ValueInputIteratorT  d_values_in,
                              KeyOutputIteratorT   d_keys_out,
                              ValueOutputIteratorT d_values_out,
                              NumSelectedIteratorT d_num_selected_out,
                              int                  num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSelect::UniqueByKey(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_values_in, d_keys_out, d_values_out, d_num_selected_out, num_items, _stream));
    }
#endif
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"