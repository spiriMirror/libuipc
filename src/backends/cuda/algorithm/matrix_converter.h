#pragma once
#include <type_define.h>
#include <muda/buffer/device_buffer.h>
#include <muda/ext/linear_system/matrix_format_converter.h>

namespace uipc::backend::cuda
{
template <typename T, int N>
class MatrixConverter
{
    using BlockMatrix   = muda::DeviceTripletMatrix<T, N>::ValueT;
    using SegmentVector = muda::DeviceDoubletVector<T, N>::ValueT;

    Float m_reserve_ratio = 1.5;

    muda::DeviceBuffer<int> col_counts_per_row;
    muda::DeviceBuffer<int> unique_indices;
    muda::DeviceBuffer<int> unique_counts;
    muda::DeviceVar<int>    count;

    muda::DeviceBuffer<int> sort_index_input;
    muda::DeviceBuffer<int> sort_index;

    muda::DeviceBuffer<int> offsets;

    muda::DeviceBuffer<int2> ij_pairs;
    muda::DeviceBuffer<int2> unique_ij_pairs;

    muda::DeviceBuffer<uint64_t> ij_hash_input;
    muda::DeviceBuffer<uint64_t> ij_hash;

    muda::DeviceBuffer<BlockMatrix> blocks_sorted;
    muda::DeviceBuffer<BlockMatrix> diag_blocks;


    muda::DeviceBuffer<int>           indices_sorted;
    muda::DeviceBuffer<SegmentVector> segments_sorted;


    muda::DeviceBuffer<int> sorted_partition_input;
    muda::DeviceBuffer<int> sorted_partition_output;

  public:
    void  reserve_ratio(Float ratio) { m_reserve_ratio = ratio; }
    Float reserve_ratio() const { return m_reserve_ratio; }


    // Triplet -> BCOO
    void convert(const muda::DeviceTripletMatrix<T, N>& from,
                 muda::DeviceBCOOMatrix<T, N>&          to);

    void _radix_sort_indices_and_blocks(const muda::DeviceTripletMatrix<T, N>& from,
                                        muda::DeviceBCOOMatrix<T, N>& to);

    void _radix_sort_indices_and_blocks(muda::DeviceBCOOMatrix<T, N>& to);

    void _make_unique_indices(const muda::DeviceTripletMatrix<T, N>& from,
                              muda::DeviceBCOOMatrix<T, N>&          to);

    void _make_unique_block_warp_reduction(const muda::DeviceTripletMatrix<T, N>& from,
                                           muda::DeviceBCOOMatrix<T, N>& to);

    // BCOO -> BSR
    void convert(const muda::DeviceBCOOMatrix<T, N>& from,
                 muda::DeviceBSRMatrix<T, N>&        to);

    void _calculate_block_offsets(const muda::DeviceBCOOMatrix<T, N>& from,
                                  muda::DeviceBSRMatrix<T, N>&        to);


    // Doublet -> BCOO
    void convert(const muda::DeviceDoubletVector<T, N>& from,
                 muda::DeviceBCOOVector<T, N>&          to);

    void _radix_sort_indices_and_segments(const muda::DeviceDoubletVector<T, N>& from,
                                          muda::DeviceBCOOVector<T, N>& to);

    void _make_unique_indices(const muda::DeviceDoubletVector<T, N>& from,
                              muda::DeviceBCOOVector<T, N>&          to);

    void _make_unique_segment_warp_reduction(const muda::DeviceDoubletVector<T, N>& from,
                                             muda::DeviceBCOOVector<T, N>& to);


    template <typename U>
    void loose_resize(muda::DeviceBuffer<U>& buf, size_t new_size)
    {
        if(buf.capacity() < new_size)
            buf.reserve(new_size * m_reserve_ratio);
        buf.resize(new_size);
    }

    void ge2sym(muda::DeviceBCOOMatrix<T, N>& to);

    void ge2sym(muda::DeviceTripletMatrix<T, N>& to);

    void sym2ge(const muda::DeviceBCOOMatrix<T, N>& from,
                muda::DeviceBCOOMatrix<T, N>&       to);
};
}  // namespace uipc::backend::cuda

#include "details/matrix_converter.inl"