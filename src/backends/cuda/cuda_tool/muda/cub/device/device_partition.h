#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_partition.cuh>
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_partition.html
class DevicePartition : public CubWrapper<DevicePartition>
{
    using Base = CubWrapper<DevicePartition>;

  public:
    using Base::Base;

    // DeviceVector:

    template <typename InputIteratorT, typename FlagIterator, typename OutputIteratorT, typename NumSelectedIteratorT>
    DevicePartition& Flagged(InputIteratorT       d_in,
                             FlagIterator         d_flags,
                             OutputIteratorT      d_out,
                             NumSelectedIteratorT d_num_selected_out,
                             int                  num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DevicePartition::Flagged(
            d_temp_storage, temp_storage_bytes, d_in, d_flags, d_out, d_num_selected_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename NumSelectedIteratorT, typename SelectOp>
    DevicePartition& If(InputIteratorT       d_in,
                        OutputIteratorT      d_out,
                        NumSelectedIteratorT d_num_selected_out,
                        int                  num_items,
                        SelectOp             select_op)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DevicePartition::If(
            d_temp_storage, temp_storage_bytes, d_in, d_out, d_num_selected_out, num_items, select_op, _stream));
    }

    template <typename InputIteratorT, typename FirstOutputIteratorT, typename SecondOutputIteratorT, typename UnselectedOutputIteratorT, typename NumSelectedIteratorT, typename SelectFirstPartOp, typename SelectSecondPartOp>
    DevicePartition& If(InputIteratorT            d_in,
                        FirstOutputIteratorT      d_first_part_out,
                        SecondOutputIteratorT     d_second_part_out,
                        UnselectedOutputIteratorT d_unselected_out,
                        NumSelectedIteratorT      d_num_selected_out,
                        int                       num_items,
                        SelectFirstPartOp         select_first_part_op,
                        SelectSecondPartOp        select_second_part_op)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DevicePartition::If(d_temp_storage,
                                                       temp_storage_bytes,
                                                       d_in,
                                                       d_first_part_out,
                                                       d_second_part_out,
                                                       d_unselected_out,
                                                       d_num_selected_out,
                                                       num_items,
                                                       select_first_part_op,
                                                       select_second_part_op,
                                                       _stream));
    }

    // Origin:

    template <typename InputIteratorT, typename FlagIterator, typename OutputIteratorT, typename NumSelectedIteratorT>
    DevicePartition& Flagged(void*                d_temp_storage,
                             size_t&              temp_storage_bytes,
                             InputIteratorT       d_in,
                             FlagIterator         d_flags,
                             OutputIteratorT      d_out,
                             NumSelectedIteratorT d_num_selected_out,
                             int                  num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DevicePartition::Flagged(
            d_temp_storage, temp_storage_bytes, d_in, d_flags, d_out, d_num_selected_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename NumSelectedIteratorT, typename SelectOp>
    DevicePartition& If(void*                d_temp_storage,
                        size_t&              temp_storage_bytes,
                        InputIteratorT       d_in,
                        OutputIteratorT      d_out,
                        NumSelectedIteratorT d_num_selected_out,
                        int                  num_items,
                        SelectOp             select_op)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DevicePartition::If(
            d_temp_storage, temp_storage_bytes, d_in, d_out, d_num_selected_out, num_items, select_op, _stream));
    }

    template <typename InputIteratorT, typename FirstOutputIteratorT, typename SecondOutputIteratorT, typename UnselectedOutputIteratorT, typename NumSelectedIteratorT, typename SelectFirstPartOp, typename SelectSecondPartOp>
    DevicePartition& If(void*                     d_temp_storage,
                        size_t&                   temp_storage_bytes,
                        InputIteratorT            d_in,
                        FirstOutputIteratorT      d_first_part_out,
                        SecondOutputIteratorT     d_second_part_out,
                        UnselectedOutputIteratorT d_unselected_out,
                        NumSelectedIteratorT      d_num_selected_out,
                        int                       num_items,
                        SelectFirstPartOp         select_first_part_op,
                        SelectSecondPartOp        select_second_part_op)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DevicePartition::If(d_temp_storage,
                                     temp_storage_bytes,
                                     d_in,
                                     d_first_part_out,
                                     d_second_part_out,
                                     d_unselected_out,
                                     d_num_selected_out,
                                     num_items,
                                     select_first_part_op,
                                     select_second_part_op,
                                     _stream));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"