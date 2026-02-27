#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_segmented_radix_sort.cuh>
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_spmv.html
class DeviceSegmentedRadixSort : public CubWrapper<DeviceSegmentedRadixSort>
{
    using Base = CubWrapper<DeviceSegmentedRadixSort>;

  public:
    using Base::Base;

    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairs(const KeyT*          d_keys_in,
                                        KeyT*                d_keys_out,
                                        const ValueT*        d_values_in,
                                        ValueT*              d_values_out,
                                        int                  num_items,
                                        int                  num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets,
                                        int                  begin_bit,
                                        int                  end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedRadixSort::SortPairs(d_temp_storage,
                                                     temp_storage_bytes,
                                                     d_keys_in,
                                                     d_keys_out,
                                                     d_values_in,
                                                     d_values_out,
                                                     num_items,
                                                     num_segments,
                                                     d_begin_offsets,
                                                     d_end_offsets,
                                                     begin_bit,
                                                     end_bit,
                                                     _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairs(cub::DoubleBuffer<KeyT>&   d_keys,
                                        cub::DoubleBuffer<ValueT>& d_values,
                                        int                        num_items,
                                        int                        num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets,
                                        int                  begin_bit,
                                        int                  end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedRadixSort::SortPairs(d_temp_storage,
                                                                       temp_storage_bytes,
                                                                       d_keys,
                                                                       d_values,
                                                                       num_items,
                                                                       num_segments,
                                                                       d_begin_offsets,
                                                                       d_end_offsets,
                                                                       begin_bit,
                                                                       end_bit,
                                                                       _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairsDescending(const KeyT*   d_keys_in,
                                                  KeyT*         d_keys_out,
                                                  const ValueT* d_values_in,
                                                  ValueT*       d_values_out,
                                                  int           num_items,
                                                  int           num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets,
                                                  int begin_bit,
                                                  int end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedRadixSort::SortPairsDescending(d_temp_storage,
                                                               temp_storage_bytes,
                                                               d_keys_in,
                                                               d_keys_out,
                                                               d_values_in,
                                                               d_values_out,
                                                               num_items,
                                                               num_segments,
                                                               d_begin_offsets,
                                                               d_end_offsets,
                                                               begin_bit,
                                                               end_bit,
                                                               _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairsDescending(cub::DoubleBuffer<KeyT>& d_keys,
                                                  cub::DoubleBuffer<ValueT>& d_values,
                                                  int num_items,
                                                  int num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets,
                                                  int begin_bit,
                                                  int end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedRadixSort::SortPairsDescending(d_temp_storage,
                                                               temp_storage_bytes,
                                                               d_keys,
                                                               d_values,
                                                               num_items,
                                                               num_segments,
                                                               d_begin_offsets,
                                                               d_end_offsets,
                                                               begin_bit,
                                                               end_bit,
                                                               _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeys(const KeyT*          d_keys_in,
                                       KeyT*                d_keys_out,
                                       int                  num_items,
                                       int                  num_segments,
                                       BeginOffsetIteratorT d_begin_offsets,
                                       EndOffsetIteratorT   d_end_offsets,
                                       int                  begin_bit,
                                       int                  end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedRadixSort::SortKeys(d_temp_storage,
                                                                      temp_storage_bytes,
                                                                      d_keys_in,
                                                                      d_keys_out,
                                                                      num_items,
                                                                      num_segments,
                                                                      d_begin_offsets,
                                                                      d_end_offsets,
                                                                      begin_bit,
                                                                      end_bit,
                                                                      _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeys(cub::DoubleBuffer<KeyT>& d_keys,
                                       int                      num_items,
                                       int                      num_segments,
                                       BeginOffsetIteratorT     d_begin_offsets,
                                       EndOffsetIteratorT       d_end_offsets,
                                       int                      begin_bit,
                                       int                      end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedRadixSort::SortKeys(d_temp_storage,
                                                                      temp_storage_bytes,
                                                                      d_keys,
                                                                      num_items,
                                                                      num_segments,
                                                                      d_begin_offsets,
                                                                      d_end_offsets,
                                                                      begin_bit,
                                                                      end_bit,
                                                                      _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeysDescending(const KeyT* d_keys_in,
                                                 KeyT*       d_keys_out,
                                                 int         num_items,
                                                 int         num_segments,
                                                 BeginOffsetIteratorT d_begin_offsets,
                                                 EndOffsetIteratorT d_end_offsets,
                                                 int begin_bit,
                                                 int end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedRadixSort::SortKeysDescending(d_temp_storage,
                                                              temp_storage_bytes,
                                                              d_keys_in,
                                                              d_keys_out,
                                                              num_items,
                                                              num_segments,
                                                              d_begin_offsets,
                                                              d_end_offsets,
                                                              begin_bit,
                                                              end_bit,
                                                              _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeysDescending(cub::DoubleBuffer<KeyT>& d_keys,
                                                 int num_items,
                                                 int num_segments,
                                                 BeginOffsetIteratorT d_begin_offsets,
                                                 EndOffsetIteratorT d_end_offsets,
                                                 int begin_bit,
                                                 int end_bit)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedRadixSort::SortKeysDescending(d_temp_storage,
                                                              temp_storage_bytes,
                                                              d_keys,
                                                              num_items,
                                                              num_segments,
                                                              d_begin_offsets,
                                                              d_end_offsets,
                                                              begin_bit,
                                                              end_bit,
                                                              _stream));
    }


    // Origin:

    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairs(void*                d_temp_storage,
                                        size_t&              temp_storage_bytes,
                                        const KeyT*          d_keys_in,
                                        KeyT*                d_keys_out,
                                        const ValueT*        d_values_in,
                                        ValueT*              d_values_out,
                                        int                  num_items,
                                        int                  num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets,
                                        int                  begin_bit,
                                        int                  end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortPairs(d_temp_storage,
                                                     temp_storage_bytes,
                                                     d_keys_in,
                                                     d_keys_out,
                                                     d_values_in,
                                                     d_values_out,
                                                     num_items,
                                                     num_segments,
                                                     d_begin_offsets,
                                                     d_end_offsets,
                                                     begin_bit,
                                                     end_bit,
                                                     _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairs(void*   d_temp_storage,
                                        size_t& temp_storage_bytes,
                                        cub::DoubleBuffer<KeyT>&   d_keys,
                                        cub::DoubleBuffer<ValueT>& d_values,
                                        int                        num_items,
                                        int                        num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets,
                                        int                  begin_bit,
                                        int                  end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortPairs(d_temp_storage,
                                                     temp_storage_bytes,
                                                     d_keys,
                                                     d_values,
                                                     num_items,
                                                     num_segments,
                                                     d_begin_offsets,
                                                     d_end_offsets,
                                                     begin_bit,
                                                     end_bit,
                                                     _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairsDescending(void*   d_temp_storage,
                                                  size_t& temp_storage_bytes,
                                                  const KeyT*   d_keys_in,
                                                  KeyT*         d_keys_out,
                                                  const ValueT* d_values_in,
                                                  ValueT*       d_values_out,
                                                  int           num_items,
                                                  int           num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets,
                                                  int begin_bit,
                                                  int end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortPairsDescending(d_temp_storage,
                                                               temp_storage_bytes,
                                                               d_keys_in,
                                                               d_keys_out,
                                                               d_values_in,
                                                               d_values_out,
                                                               num_items,
                                                               num_segments,
                                                               d_begin_offsets,
                                                               d_end_offsets,
                                                               begin_bit,
                                                               end_bit,
                                                               _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortPairsDescending(void*   d_temp_storage,
                                                  size_t& temp_storage_bytes,
                                                  cub::DoubleBuffer<KeyT>& d_keys,
                                                  cub::DoubleBuffer<ValueT>& d_values,
                                                  int num_items,
                                                  int num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets,
                                                  int begin_bit,
                                                  int end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortPairsDescending(d_temp_storage,
                                                               temp_storage_bytes,
                                                               d_keys,
                                                               d_values,
                                                               num_items,
                                                               num_segments,
                                                               d_begin_offsets,
                                                               d_end_offsets,
                                                               begin_bit,
                                                               end_bit,
                                                               _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeys(void*                d_temp_storage,
                                       size_t&              temp_storage_bytes,
                                       const KeyT*          d_keys_in,
                                       KeyT*                d_keys_out,
                                       int                  num_items,
                                       int                  num_segments,
                                       BeginOffsetIteratorT d_begin_offsets,
                                       EndOffsetIteratorT   d_end_offsets,
                                       int                  begin_bit,
                                       int                  end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortKeys(d_temp_storage,
                                                    temp_storage_bytes,
                                                    d_keys_in,
                                                    d_keys_out,
                                                    num_items,
                                                    num_segments,
                                                    d_begin_offsets,
                                                    d_end_offsets,
                                                    begin_bit,
                                                    end_bit,
                                                    _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeys(void*   d_temp_storage,
                                       size_t& temp_storage_bytes,
                                       cub::DoubleBuffer<KeyT>& d_keys,
                                       int                      num_items,
                                       int                      num_segments,
                                       BeginOffsetIteratorT     d_begin_offsets,
                                       EndOffsetIteratorT       d_end_offsets,
                                       int                      begin_bit,
                                       int                      end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortKeys(d_temp_storage,
                                                    temp_storage_bytes,
                                                    d_keys,
                                                    num_items,
                                                    num_segments,
                                                    d_begin_offsets,
                                                    d_end_offsets,
                                                    begin_bit,
                                                    end_bit,
                                                    _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeysDescending(void*       d_temp_storage,
                                                 size_t&     temp_storage_bytes,
                                                 const KeyT* d_keys_in,
                                                 KeyT*       d_keys_out,
                                                 int         num_items,
                                                 int         num_segments,
                                                 BeginOffsetIteratorT d_begin_offsets,
                                                 EndOffsetIteratorT d_end_offsets,
                                                 int begin_bit,
                                                 int end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortKeysDescending(d_temp_storage,
                                                              temp_storage_bytes,
                                                              d_keys_in,
                                                              d_keys_out,
                                                              num_items,
                                                              num_segments,
                                                              d_begin_offsets,
                                                              d_end_offsets,
                                                              begin_bit,
                                                              end_bit,
                                                              _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedRadixSort& SortKeysDescending(void*   d_temp_storage,
                                                 size_t& temp_storage_bytes,
                                                 cub::DoubleBuffer<KeyT>& d_keys,
                                                 int num_items,
                                                 int num_segments,
                                                 BeginOffsetIteratorT d_begin_offsets,
                                                 EndOffsetIteratorT d_end_offsets,
                                                 int begin_bit,
                                                 int end_bit)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedRadixSort::SortKeysDescending(d_temp_storage,
                                                              temp_storage_bytes,
                                                              d_keys,
                                                              num_items,
                                                              num_segments,
                                                              d_begin_offsets,
                                                              d_end_offsets,
                                                              begin_bit,
                                                              end_bit,
                                                              _stream));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"