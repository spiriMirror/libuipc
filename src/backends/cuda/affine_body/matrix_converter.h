#pragma once
#include <type_define.h>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_var.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>
#include <muda/ext/linear_system/device_bcoo_matrix.h>
namespace uipc::backend::cuda
{
class ABDMatrixConverter
{
  public:
    using T                = Float;
    constexpr static int N = 12;
    using BlockMatrix      = muda::DeviceTripletMatrix<T, N>::ValueT;

    class Impl
    {
      public:
        // Triplet -> BCOO
        void convert(const muda::DeviceTripletMatrix<T, N>& from,
                     muda::DeviceBCOOMatrix<T, N>&          to);

        void _radix_sort_indices_and_blocks(const muda::DeviceTripletMatrix<T, N>& from,
                                            muda::DeviceBCOOMatrix<T, N>& to);
        void _make_unique_indices(const muda::DeviceTripletMatrix<T, N>& from,
                                  muda::DeviceBCOOMatrix<T, N>&          to);
        void _make_unique_block_warp_reduction(const muda::DeviceTripletMatrix<T, N>& from,
                                               muda::DeviceBCOOMatrix<T, N>& to);

        template <typename T>
        void loose_resize(muda::DeviceBuffer<T>& buf, size_t new_size)
        {
            if(buf.capacity() < new_size)
                buf.reserve(new_size * reserve_ratio);
            buf.resize(new_size);
        }

        muda::DeviceBuffer<int> unique_counts;
        muda::DeviceVar<int>    count;

        muda::DeviceBuffer<int> sort_index;
        muda::DeviceBuffer<int> sort_index_input;

        muda::DeviceBuffer<int> offsets;

        muda::DeviceBuffer<int2>     ij_pairs;
        muda::DeviceBuffer<uint64_t> ij_hash;
        muda::DeviceBuffer<uint64_t> ij_hash_input;
        muda::DeviceBuffer<int2>     unique_ij_pairs;

        muda::DeviceBuffer<BlockMatrix> blocks_sorted;

        Float reserve_ratio = 1.1;

        muda::DeviceBuffer<int> sorted_partition_input;
        muda::DeviceBuffer<int> sorted_partition_output;
    };

    void convert(const muda::DeviceTripletMatrix<T, N>& from,
                 muda::DeviceBCOOMatrix<T, N>&          to);

    void reserve_ratio(Float ratio) noexcept { m_impl.reserve_ratio = ratio; }

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
