#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#ifndef __INTELLISENSE__
#include <cub/device/device_scan.cuh>
#else
namespace cub
{
class Equality
{
    //dummy class just for Intellisense
};
}  // namespace cub
#endif


namespace muda
{
#if CUDA_VERSION < 13000
#define MUDA_EQUALITY cub::Equality
#else
#define MUDA_EQUALITY ::cuda::std::equal_to<>
#endif
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_scan.html
class DeviceScan : public CubWrapper<DeviceScan>
{
    using Base = CubWrapper<DeviceScan>;

  public:
    using Base::Base;

    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceScan& ExclusiveSum(InputIteratorT d_in, OutputIteratorT d_out, int num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::ExclusiveSum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT, typename ScanOpT, typename InitValueT>
    DeviceScan& ExclusiveScan(InputIteratorT  d_in,
                              OutputIteratorT d_out,
                              ScanOpT         scan_op,
                              InitValueT      init_value,
                              int             num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::ExclusiveScan(
            d_temp_storage, temp_storage_bytes, d_in, d_out, scan_op, init_value, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceScan& InclusiveSum(InputIteratorT d_in, OutputIteratorT d_out, int num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::InclusiveSum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename ScanOpT>
    DeviceScan& InclusiveScan(InputIteratorT d_in, OutputIteratorT d_out, ScanOpT scan_op, int num_items)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::InclusiveScan(
            d_temp_storage, temp_storage_bytes, d_in, d_out, scan_op, num_items, _stream));
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& ExclusiveSumByKey(KeysInputIteratorT    d_keys_in,
                                  ValuesInputIteratorT  d_values_in,
                                  ValuesOutputIteratorT d_values_out,
                                  int                   num_items,
                                  EqualityOpT equality_op = EqualityOpT())
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::ExclusiveSumByKey(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_values_in, d_values_out, num_items, equality_op, _stream));
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename ScanOpT, typename InitValueT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& ExclusiveScanByKey(KeysInputIteratorT    d_keys_in,
                                   ValuesInputIteratorT  d_values_in,
                                   ValuesOutputIteratorT d_values_out,
                                   ScanOpT               scan_op,
                                   InitValueT            init_value,
                                   int                   num_items,
                                   EqualityOpT equality_op = EqualityOpT())
    {
#if CUDA_VERSION < 13000
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::ExclusiveScanByKey(d_temp_storage,
                                                                  temp_storage_bytes,
                                                                  d_keys_in,
                                                                  d_values_in,
                                                                  d_values_out,
                                                                  scan_op,
                                                                  init_value,
                                                                  num_items,
                                                                  equality_op,
                                                                  _stream ,false
                                                                  ));
#else
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::ExclusiveScanByKey(d_temp_storage,
                                                                  temp_storage_bytes,
                                                                  d_keys_in,
                                                                  d_values_in,
                                                                  d_values_out,
                                                                  scan_op,
                                                                  init_value,
                                                                  num_items,
                                                                  equality_op,
                                                                  _stream
                                                                  ));
#endif
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& InclusiveSumByKey(KeysInputIteratorT    d_keys_in,
                                  ValuesInputIteratorT  d_values_in,
                                  ValuesOutputIteratorT d_values_out,
                                  int                   num_items,
                                  EqualityOpT equality_op = EqualityOpT())
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::InclusiveSumByKey(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_values_in, d_values_out, num_items, equality_op, _stream));
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename ScanOpT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& InclusiveScanByKey(KeysInputIteratorT    d_keys_in,
                                   ValuesInputIteratorT  d_values_in,
                                   ValuesOutputIteratorT d_values_out,
                                   ScanOpT               scan_op,
                                   int                   num_items,
                                   EqualityOpT equality_op = EqualityOpT())
    {
#if CUDA_VERSION < 13000
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::InclusiveScanByKey(d_temp_storage,
                                                                  temp_storage_bytes,
                                                                  d_keys_in,
                                                                  d_values_in,
                                                                  d_values_out,
                                                                  scan_op,
                                                                  num_items,
                                                                  equality_op,
                                                                  _stream,
                                                                  false));
#else
MUDA_CUB_WRAPPER_IMPL(cub::DeviceScan::InclusiveScanByKey(d_temp_storage,
                                                                  temp_storage_bytes,
                                                                  d_keys_in,
                                                                  d_values_in,
                                                                  d_values_out,
                                                                  scan_op,
                                                                  num_items,
                                                                  equality_op,
                                                                  _stream));
#endif
    }

    // Origin:

    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceScan& ExclusiveSum(void*           d_temp_storage,
                             size_t&         temp_storage_bytes,
                             InputIteratorT  d_in,
                             OutputIteratorT d_out,
                             int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceScan::ExclusiveSum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT, typename ScanOpT, typename InitValueT>
    DeviceScan& ExclusiveScan(void*           d_temp_storage,
                              size_t&         temp_storage_bytes,
                              InputIteratorT  d_in,
                              OutputIteratorT d_out,
                              ScanOpT         scan_op,
                              InitValueT      init_value,
                              int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceScan::ExclusiveScan(
            d_temp_storage, temp_storage_bytes, d_in, d_out, scan_op, init_value, num_items, _stream));
    }


    template <typename InputIteratorT, typename OutputIteratorT>
    DeviceScan& InclusiveSum(void*           d_temp_storage,
                             size_t&         temp_storage_bytes,
                             InputIteratorT  d_in,
                             OutputIteratorT d_out,
                             int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceScan::InclusiveSum(
            d_temp_storage, temp_storage_bytes, d_in, d_out, num_items, _stream));
    }

    template <typename InputIteratorT, typename OutputIteratorT, typename ScanOpT>
    DeviceScan& InclusiveScan(void*           d_temp_storage,
                              size_t&         temp_storage_bytes,
                              InputIteratorT  d_in,
                              OutputIteratorT d_out,
                              ScanOpT         scan_op,
                              int             num_items)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceScan::InclusiveScan(
            d_temp_storage, temp_storage_bytes, d_in, d_out, scan_op, num_items, _stream));
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& ExclusiveSumByKey(void*                 d_temp_storage,
                                  size_t&               temp_storage_bytes,
                                  KeysInputIteratorT    d_keys_in,
                                  ValuesInputIteratorT  d_values_in,
                                  ValuesOutputIteratorT d_values_out,
                                  int                   num_items,
                                  EqualityOpT equality_op = EqualityOpT())
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceScan::ExclusiveSumByKey(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_values_in, d_values_out, num_items, equality_op, _stream));
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename ScanOpT, typename InitValueT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& ExclusiveScanByKey(void*                 d_temp_storage,
                                   size_t&               temp_storage_bytes,
                                   KeysInputIteratorT    d_keys_in,
                                   ValuesInputIteratorT  d_values_in,
                                   ValuesOutputIteratorT d_values_out,
                                   ScanOpT               scan_op,
                                   InitValueT            init_value,
                                   int                   num_items,
                                   EqualityOpT equality_op = EqualityOpT())
    {
#if CUDA_VERSION < 13000
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceScan::ExclusiveScanByKey(d_temp_storage,
                                                temp_storage_bytes,
                                                d_keys_in,
                                                d_values_in,
                                                d_values_out,
                                                scan_op,
                                                init_value,
                                                num_items,
                                                equality_op,
                                                _stream,
                                                false));
#else
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(
            cub::DeviceScan::ExclusiveScanByKey(d_temp_storage,
                                                temp_storage_bytes,
                                                d_keys_in,
                                                d_values_in,
                                                d_values_out,
                                                scan_op,
                                                init_value,
                                                num_items,
                                                equality_op,
                                                _stream));
#endif
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& InclusiveSumByKey(void*                 d_temp_storage,
                                  size_t&               temp_storage_bytes,
                                  KeysInputIteratorT    d_keys_in,
                                  ValuesInputIteratorT  d_values_in,
                                  ValuesOutputIteratorT d_values_out,
                                  int                   num_items,
                                  EqualityOpT equality_op = EqualityOpT())
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceScan::InclusiveSumByKey(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_values_in, d_values_out, num_items, equality_op, _stream));
    }

    template <typename KeysInputIteratorT, typename ValuesInputIteratorT, typename ValuesOutputIteratorT, typename ScanOpT, typename EqualityOpT = MUDA_EQUALITY>
    DeviceScan& InclusiveScanByKey(void*                 d_temp_storage,
                                   size_t&               temp_storage_bytes,
                                   KeysInputIteratorT    d_keys_in,
                                   ValuesInputIteratorT  d_values_in,
                                   ValuesOutputIteratorT d_values_out,
                                   ScanOpT               scan_op,
                                   int                   num_items,
                                   EqualityOpT equality_op = EqualityOpT())
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceScan::InclusiveScanByKey(
            d_temp_storage, temp_storage_bytes, d_keys_in, d_values_in, d_values_out, scan_op, num_items, equality_op, _stream));
    }
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"