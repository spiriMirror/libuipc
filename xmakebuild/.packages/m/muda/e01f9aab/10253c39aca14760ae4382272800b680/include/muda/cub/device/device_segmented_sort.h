#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_segmented_sort.cuh>
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_segmented_reduce.html
class DeviceSegmentedSort : public CubWrapper<DeviceSegmentedSort>
{
    using Base = CubWrapper<DeviceSegmentedSort>;

  public:
    using Base::Base;

    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeys(const KeyT*          d_keys_in,
                                  KeyT*                d_keys_out,
                                  int                  num_items,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::SortKeys(d_temp_storage,
                                                                 temp_storage_bytes,
                                                                 d_keys_in,
                                                                 d_keys_out,
                                                                 num_items,
                                                                 num_segments,
                                                                 d_begin_offsets,
                                                                 d_end_offsets,
                                                                 _stream));
    }

    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeysDescending(const KeyT* d_keys_in,
                                            KeyT*       d_keys_out,
                                            int         num_items,
                                            int         num_segments,
                                            BeginOffsetIteratorT d_begin_offsets,
                                            EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeys(cub::DoubleBuffer<KeyT>& d_keys,
                                  int                      num_items,
                                  int                      num_segments,
                                  BeginOffsetIteratorT     d_begin_offsets,
                                  EndOffsetIteratorT       d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::SortKeys(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeysDescending(cub::DoubleBuffer<KeyT>& d_keys,
                                            int                      num_items,
                                            int num_segments,
                                            BeginOffsetIteratorT d_begin_offsets,
                                            EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeys(const KeyT*          d_keys_in,
                                        KeyT*                d_keys_out,
                                        int                  num_items,
                                        int                  num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::StableSortKeys(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeysDescending(const KeyT* d_keys_in,
                                                  KeyT*       d_keys_out,
                                                  int         num_items,
                                                  int         num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::StableSortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeys(cub::DoubleBuffer<KeyT>& d_keys,
                                        int                      num_items,
                                        int                      num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::StableSortKeys(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeysDescending(cub::DoubleBuffer<KeyT>& d_keys,
                                                  int num_items,
                                                  int num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::StableSortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairs(const KeyT*          d_keys_in,
                                   KeyT*                d_keys_out,
                                   const ValueT*        d_values_in,
                                   ValueT*              d_values_out,
                                   int                  num_items,
                                   int                  num_segments,
                                   BeginOffsetIteratorT d_begin_offsets,
                                   EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::SortPairs(d_temp_storage,
                                                                  temp_storage_bytes,
                                                                  d_keys_in,
                                                                  d_keys_out,
                                                                  d_values_in,
                                                                  d_values_out,
                                                                  num_items,
                                                                  num_segments,
                                                                  d_begin_offsets,
                                                                  d_end_offsets,
                                                                  _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairsDescending(const KeyT*   d_keys_in,
                                             KeyT*         d_keys_out,
                                             const ValueT* d_values_in,
                                             ValueT*       d_values_out,
                                             int           num_items,
                                             int           num_segments,
                                             BeginOffsetIteratorT d_begin_offsets,
                                             EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedSort::SortPairsDescending(d_temp_storage,
                                                          temp_storage_bytes,
                                                          d_keys_in,
                                                          d_keys_out,
                                                          d_values_in,
                                                          d_values_out,
                                                          num_items,
                                                          num_segments,
                                                          d_begin_offsets,
                                                          d_end_offsets,
                                                          _stream));
    }

    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairs(cub::DoubleBuffer<KeyT>&   d_keys,
                                   cub::DoubleBuffer<ValueT>& d_values,
                                   int                        num_items,
                                   int                        num_segments,
                                   BeginOffsetIteratorT       d_begin_offsets,
                                   EndOffsetIteratorT         d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::SortPairs(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairsDescending(cub::DoubleBuffer<KeyT>& d_keys,
                                             cub::DoubleBuffer<ValueT>& d_values,
                                             int num_items,
                                             int num_segments,
                                             BeginOffsetIteratorT d_begin_offsets,
                                             EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::SortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairs(const KeyT*          d_keys_in,
                                         KeyT*                d_keys_out,
                                         const ValueT*        d_values_in,
                                         ValueT*              d_values_out,
                                         int                  num_items,
                                         int                  num_segments,
                                         BeginOffsetIteratorT d_begin_offsets,
                                         EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedSort::StableSortPairs(d_temp_storage,
                                                      temp_storage_bytes,
                                                      d_keys_in,
                                                      d_keys_out,
                                                      d_values_in,
                                                      d_values_out,
                                                      num_items,
                                                      num_segments,
                                                      d_begin_offsets,
                                                      d_end_offsets,
                                                      _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairsDescending(const KeyT*   d_keys_in,
                                                   KeyT*         d_keys_out,
                                                   const ValueT* d_values_in,
                                                   ValueT*       d_values_out,
                                                   int           num_items,
                                                   int           num_segments,
                                                   BeginOffsetIteratorT d_begin_offsets,
                                                   EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(
            cub::DeviceSegmentedSort::StableSortPairsDescending(d_temp_storage,
                                                                temp_storage_bytes,
                                                                d_keys_in,
                                                                d_keys_out,
                                                                d_values_in,
                                                                d_values_out,
                                                                num_items,
                                                                num_segments,
                                                                d_begin_offsets,
                                                                d_end_offsets,
                                                                _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairs(cub::DoubleBuffer<KeyT>&   d_keys,
                                         cub::DoubleBuffer<ValueT>& d_values,
                                         int                        num_items,
                                         int                  num_segments,
                                         BeginOffsetIteratorT d_begin_offsets,
                                         EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::StableSortPairs(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairsDescending(cub::DoubleBuffer<KeyT>& d_keys,
                                                   cub::DoubleBuffer<ValueT>& d_values,
                                                   int num_items,
                                                   int num_segments,
                                                   BeginOffsetIteratorT d_begin_offsets,
                                                   EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSegmentedSort::StableSortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    // Origin:

    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeys(void*                d_temp_storage,
                                  size_t&              temp_storage_bytes,
                                  const KeyT*          d_keys_in,
                                  KeyT*                d_keys_out,
                                  int                  num_items,
                                  int                  num_segments,
                                  BeginOffsetIteratorT d_begin_offsets,
                                  EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::SortKeys(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }

    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeysDescending(void*       d_temp_storage,
                                            size_t&     temp_storage_bytes,
                                            const KeyT* d_keys_in,
                                            KeyT*       d_keys_out,
                                            int         num_items,
                                            int         num_segments,
                                            BeginOffsetIteratorT d_begin_offsets,
                                            EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeys(void*                    d_temp_storage,
                                  size_t&                  temp_storage_bytes,
                                  cub::DoubleBuffer<KeyT>& d_keys,
                                  int                      num_items,
                                  int                      num_segments,
                                  BeginOffsetIteratorT     d_begin_offsets,
                                  EndOffsetIteratorT       d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::SortKeys(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortKeysDescending(void*   d_temp_storage,
                                            size_t& temp_storage_bytes,
                                            cub::DoubleBuffer<KeyT>& d_keys,
                                            int                      num_items,
                                            int num_segments,
                                            BeginOffsetIteratorT d_begin_offsets,
                                            EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeys(void*                d_temp_storage,
                                        size_t&              temp_storage_bytes,
                                        const KeyT*          d_keys_in,
                                        KeyT*                d_keys_out,
                                        int                  num_items,
                                        int                  num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::StableSortKeys(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeysDescending(void*   d_temp_storage,
                                                  size_t& temp_storage_bytes,
                                                  const KeyT* d_keys_in,
                                                  KeyT*       d_keys_out,
                                                  int         num_items,
                                                  int         num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::StableSortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeys(void*   d_temp_storage,
                                        size_t& temp_storage_bytes,
                                        cub::DoubleBuffer<KeyT>& d_keys,
                                        int                      num_items,
                                        int                      num_segments,
                                        BeginOffsetIteratorT d_begin_offsets,
                                        EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::StableSortKeys(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortKeysDescending(void*   d_temp_storage,
                                                  size_t& temp_storage_bytes,
                                                  cub::DoubleBuffer<KeyT>& d_keys,
                                                  int num_items,
                                                  int num_segments,
                                                  BeginOffsetIteratorT d_begin_offsets,
                                                  EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::StableSortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairs(void*                d_temp_storage,
                                   size_t&              temp_storage_bytes,
                                   const KeyT*          d_keys_in,
                                   KeyT*                d_keys_out,
                                   const ValueT*        d_values_in,
                                   ValueT*              d_values_out,
                                   int                  num_items,
                                   int                  num_segments,
                                   BeginOffsetIteratorT d_begin_offsets,
                                   EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedSort::SortPairs(d_temp_storage,
                                                temp_storage_bytes,
                                                d_keys_in,
                                                d_keys_out,
                                                d_values_in,
                                                d_values_out,
                                                num_items,
                                                num_segments,
                                                d_begin_offsets,
                                                d_end_offsets,
                                                _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairsDescending(void*         d_temp_storage,
                                             size_t&       temp_storage_bytes,
                                             const KeyT*   d_keys_in,
                                             KeyT*         d_keys_out,
                                             const ValueT* d_values_in,
                                             ValueT*       d_values_out,
                                             int           num_items,
                                             int           num_segments,
                                             BeginOffsetIteratorT d_begin_offsets,
                                             EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedSort::SortPairsDescending(d_temp_storage,
                                                          temp_storage_bytes,
                                                          d_keys_in,
                                                          d_keys_out,
                                                          d_values_in,
                                                          d_values_out,
                                                          num_items,
                                                          num_segments,
                                                          d_begin_offsets,
                                                          d_end_offsets,
                                                          _stream));
    }

    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairs(void*                    d_temp_storage,
                                   size_t&                  temp_storage_bytes,
                                   cub::DoubleBuffer<KeyT>& d_keys,
                                   cub::DoubleBuffer<ValueT>& d_values,
                                   int                        num_items,
                                   int                        num_segments,
                                   BeginOffsetIteratorT       d_begin_offsets,
                                   EndOffsetIteratorT         d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::SortPairs(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& SortPairsDescending(void*   d_temp_storage,
                                             size_t& temp_storage_bytes,
                                             cub::DoubleBuffer<KeyT>& d_keys,
                                             cub::DoubleBuffer<ValueT>& d_values,
                                             int num_items,
                                             int num_segments,
                                             BeginOffsetIteratorT d_begin_offsets,
                                             EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::SortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairs(void*         d_temp_storage,
                                         size_t&       temp_storage_bytes,
                                         const KeyT*   d_keys_in,
                                         KeyT*         d_keys_out,
                                         const ValueT* d_values_in,
                                         ValueT*       d_values_out,
                                         int           num_items,
                                         int           num_segments,
                                         BeginOffsetIteratorT d_begin_offsets,
                                         EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedSort::StableSortPairs(d_temp_storage,
                                                      temp_storage_bytes,
                                                      d_keys_in,
                                                      d_keys_out,
                                                      d_values_in,
                                                      d_values_out,
                                                      num_items,
                                                      num_segments,
                                                      d_begin_offsets,
                                                      d_end_offsets,
                                                      _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairsDescending(void*   d_temp_storage,
                                                   size_t& temp_storage_bytes,
                                                   const KeyT*   d_keys_in,
                                                   KeyT*         d_keys_out,
                                                   const ValueT* d_values_in,
                                                   ValueT*       d_values_out,
                                                   int           num_items,
                                                   int           num_segments,
                                                   BeginOffsetIteratorT d_begin_offsets,
                                                   EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceSegmentedSort::StableSortPairsDescending(d_temp_storage,
                                                                temp_storage_bytes,
                                                                d_keys_in,
                                                                d_keys_out,
                                                                d_values_in,
                                                                d_values_out,
                                                                num_items,
                                                                num_segments,
                                                                d_begin_offsets,
                                                                d_end_offsets,
                                                                _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairs(void*   d_temp_storage,
                                         size_t& temp_storage_bytes,
                                         cub::DoubleBuffer<KeyT>&   d_keys,
                                         cub::DoubleBuffer<ValueT>& d_values,
                                         int                        num_items,
                                         int                  num_segments,
                                         BeginOffsetIteratorT d_begin_offsets,
                                         EndOffsetIteratorT   d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::StableSortPairs(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }


    template <typename KeyT, typename ValueT, typename BeginOffsetIteratorT, typename EndOffsetIteratorT>
    DeviceSegmentedSort& StableSortPairsDescending(void*   d_temp_storage,
                                                   size_t& temp_storage_bytes,
                                                   cub::DoubleBuffer<KeyT>& d_keys,
                                                   cub::DoubleBuffer<ValueT>& d_values,
                                                   int num_items,
                                                   int num_segments,
                                                   BeginOffsetIteratorT d_begin_offsets,
                                                   EndOffsetIteratorT d_end_offsets)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSegmentedSort::StableSortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, num_segments, d_begin_offsets, d_end_offsets, _stream));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"