#pragma once
#include <muda/cub/device/cub_wrapper.h>
#include "details/cub_wrapper_macro_def.inl"
#if CUDA_VERSION < 13000
#ifndef __INTELLISENSE__
#include <cub/device/device_spmv.cuh>
#endif
#endif

namespace muda
{
//ref: https://nvlabs.github.io/cub/structcub_1_1_device_spmv.html
class DeviceSpmv : public CubWrapper<DeviceSpmv>
{
    using Base = CubWrapper<DeviceSpmv>;

  public:
    using Base::Base;
#if CUDA_VERSION < 13000
    template <typename ValueT>
    DeviceSpmv& CsrMV(const ValueT* d_values,
                      const int*    d_row_offsets,
                      const int*    d_column_indices,
                      const ValueT* d_vector_x,
                      ValueT*       d_vector_y,
                      int           num_rows,
                      int           num_cols,
                      int           num_nonzeros)
    {
        MUDA_CUB_WRAPPER_IMPL(cub::DeviceSpmv::CsrMV(d_temp_storage,
                                                     temp_storage_bytes,
                                                     d_values,
                                                     d_row_offsets,
                                                     d_column_indices,
                                                     d_vector_x,
                                                     d_vector_y,
                                                     num_rows,
                                                     num_cols,
                                                     num_nonzeros,
                                                     _stream));
    }

    // Origin:

    template <typename ValueT>
    DeviceSpmv& CsrMV(void*         d_temp_storage,
                      size_t&       temp_storage_bytes,
                      const ValueT* d_values,
                      const int*    d_row_offsets,
                      const int*    d_column_indices,
                      const ValueT* d_vector_x,
                      ValueT*       d_vector_y,
                      int           num_rows,
                      int           num_cols,
                      int           num_nonzeros)
    {
        MUDA_CUB_WRAPPER_FOR_COMPUTE_GRAPH_IMPL(cub::DeviceSpmv::CsrMV(d_temp_storage,
                                                                       temp_storage_bytes,
                                                                       d_values,
                                                                       d_row_offsets,
                                                                       d_column_indices,
                                                                       d_vector_x,
                                                                       d_vector_y,
                                                                       num_rows,
                                                                       num_cols,
                                                                       num_nonzeros,
                                                                       _stream));
    }
  #else
  // TODO: CsrMV
  #endif
};
}  // namespace muda

#include "details/cub_wrapper_macro_undef.inl"