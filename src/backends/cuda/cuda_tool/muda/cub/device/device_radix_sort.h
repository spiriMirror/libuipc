#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_radix_sort.cuh>
#else
//namespace cub
//{
//template <typename KeyT>
//struct DoubleBuffer;
//}
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_radix_sort.html
class DeviceRadixSort : public CubWrapper<DeviceRadixSort>
{
    using Base = CubWrapper<DeviceRadixSort>;

  public:
    using Base::Base;

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairs(const KeyT*   d_keys_in,
                               KeyT*         d_keys_out,
                               const ValueT* d_values_in,
                               ValueT*       d_values_out,
                               NumItemsT     num_items,
                               int           begin_bit = 0,
                               int           end_bit   = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortPairs(d_temp_storage,
                                                              temp_storage_bytes,
                                                              d_keys_in,
                                                              d_keys_out,
                                                              d_values_in,
                                                              d_values_out,
                                                              num_items,
                                                              begin_bit,
                                                              end_bit,
                                                              _stream));
    }

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairs(DeviceVector<std::byte>&   external_buffer,
                               cub::DoubleBuffer<KeyT>&   d_keys,
                               cub::DoubleBuffer<ValueT>& d_values,
                               NumItemsT                  num_items,
                               int                        begin_bit = 0,
                               int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortPairs(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairsDescending(const KeyT*   d_keys_in,
                                         KeyT*         d_keys_out,
                                         const ValueT* d_values_in,
                                         ValueT*       d_values_out,
                                         NumItemsT     num_items,
                                         int           begin_bit = 0,
                                         int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in, d_values_out, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairsDescending(cub::DoubleBuffer<KeyT>&   d_keys,
                                         cub::DoubleBuffer<ValueT>& d_values,
                                         NumItemsT                  num_items,
                                         int begin_bit = 0,
                                         int end_bit   = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeys(const KeyT* d_keys_in,
                              KeyT*       d_keys_out,
                              NumItemsT   num_items,
                              int         begin_bit = 0,
                              int         end_bit   = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortKeys(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeys(cub::DoubleBuffer<KeyT>& d_keys,
                              NumItemsT                num_items,
                              int                      begin_bit = 0,
                              int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortKeys(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeysDescending(const KeyT* d_keys_in,
                                        KeyT*       d_keys_out,
                                        NumItemsT   num_items,
                                        int         begin_bit = 0,
                                        int         end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeysDescending(cub::DoubleBuffer<KeyT>& d_keys,
                                        NumItemsT                num_items,
                                        int                      begin_bit = 0,
                                        int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceRadixSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, begin_bit, end_bit, _stream));
    }

    // Origin:

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairs(void*         d_temp_storage,
                               size_t&       temp_storage_bytes,
                               const KeyT*   d_keys_in,
                               KeyT*         d_keys_out,
                               const ValueT* d_values_in,
                               ValueT*       d_values_out,
                               NumItemsT     num_items,
                               int           begin_bit = 0,
                               int           end_bit   = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortPairs(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in, d_values_out, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairs(void*                      d_temp_storage,
                               size_t&                    temp_storage_bytes,
                               cub::DoubleBuffer<KeyT>&   d_keys,
                               cub::DoubleBuffer<ValueT>& d_values,
                               NumItemsT                  num_items,
                               int                        begin_bit = 0,
                               int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortPairs(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairsDescending(void*         d_temp_storage,
                                         size_t&       temp_storage_bytes,
                                         const KeyT*   d_keys_in,
                                         KeyT*         d_keys_out,
                                         const ValueT* d_values_in,
                                         ValueT*       d_values_out,
                                         NumItemsT     num_items,
                                         int           begin_bit = 0,
                                         int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in, d_values_out, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename ValueT, typename NumItemsT>
    DeviceRadixSort& SortPairsDescending(void*   d_temp_storage,
                                         size_t& temp_storage_bytes,
                                         cub::DoubleBuffer<KeyT>&   d_keys,
                                         cub::DoubleBuffer<ValueT>& d_values,
                                         NumItemsT                  num_items,
                                         int begin_bit = 0,
                                         int end_bit   = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortPairsDescending(
            d_temp_storage, temp_storage_bytes, d_keys, d_values, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeys(void*       d_temp_storage,
                              size_t&     temp_storage_bytes,
                              const KeyT* d_keys_in,
                              KeyT*       d_keys_out,
                              NumItemsT   num_items,
                              int         begin_bit = 0,
                              int         end_bit   = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortKeys(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeys(void*                    d_temp_storage,
                              size_t&                  temp_storage_bytes,
                              cub::DoubleBuffer<KeyT>& d_keys,
                              NumItemsT                num_items,
                              int                      begin_bit = 0,
                              int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortKeys(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeysDescending(void*       d_temp_storage,
                                        size_t&     temp_storage_bytes,
                                        const KeyT* d_keys_in,
                                        KeyT*       d_keys_out,
                                        NumItemsT   num_items,
                                        int         begin_bit = 0,
                                        int         end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, num_items, begin_bit, end_bit, _stream));
    }

    template <typename KeyT, typename NumItemsT>
    DeviceRadixSort& SortKeysDescending(void*   d_temp_storage,
                                        size_t& temp_storage_bytes,
                                        cub::DoubleBuffer<KeyT>& d_keys,
                                        NumItemsT                num_items,
                                        int                      begin_bit = 0,
                                        int end_bit = sizeof(KeyT) * 8)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceRadixSort::SortKeysDescending(
            d_temp_storage, temp_storage_bytes, d_keys, num_items, begin_bit, end_bit, _stream));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"