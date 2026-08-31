#pragma once
#include <type_define.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
struct MatrixConverterIntPair
{
    int x;
    int y;
};

constexpr bool operator==(const MatrixConverterIntPair& l, const MatrixConverterIntPair& r)
{
    return l.x == r.x && l.y == r.y;
}

template <typename T, int N>
class MatrixConverter
{
    using BlockMatrix   = cuda_tool::DeviceTripletMatrix<T, N>::ValueT;
    using SegmentVector = cuda_tool::DeviceDoubletVector<T, N>::ValueT;

    Float m_reserve_ratio = 1.5;

    cuda_tool::DeviceBuffer<int> col_counts_per_row;
    cuda_tool::DeviceBuffer<int> unique_indices;
    cuda_tool::DeviceBuffer<int> unique_counts;
    cuda_tool::DeviceVar<int>    count;

    cuda_tool::DeviceBuffer<int> sort_index_input;
    cuda_tool::DeviceBuffer<int> sort_index;

    cuda_tool::DeviceBuffer<int> offsets;

    cuda_tool::DeviceBuffer<MatrixConverterIntPair> ij_pairs;
    cuda_tool::DeviceBuffer<MatrixConverterIntPair> unique_ij_pairs;

    cuda_tool::DeviceBuffer<uint64_t> ij_hash_input;
    cuda_tool::DeviceBuffer<uint64_t> ij_hash;

    cuda_tool::DeviceBuffer<BlockMatrix> blocks_sorted;
    cuda_tool::DeviceBuffer<BlockMatrix> diag_blocks;


    cuda_tool::DeviceBuffer<int>           indices_sorted;
    cuda_tool::DeviceBuffer<SegmentVector> segments_sorted;


    cuda_tool::DeviceBuffer<int> sorted_partition_input;
    cuda_tool::DeviceBuffer<int> sorted_partition_output;

  public:
    void  reserve_ratio(Float ratio) { m_reserve_ratio = ratio; }
    Float reserve_ratio() const { return m_reserve_ratio; }


    // Triplet -> BCOO
    void convert(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                 cuda_tool::DeviceBCOOMatrix<T, N>&          to);

    void _radix_sort_indices_and_blocks(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                        cuda_tool::DeviceBCOOMatrix<T, N>& to);

    void _radix_sort_indices_and_blocks(cuda_tool::DeviceBCOOMatrix<T, N>& to);

    void _make_unique_indices(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                              cuda_tool::DeviceBCOOMatrix<T, N>&          to);

    void _make_unique_block_warp_reduction(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                           cuda_tool::DeviceBCOOMatrix<T, N>& to);

    // BCOO -> BSR
    void convert(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                 cuda_tool::DeviceBSRMatrix<T, N>&        to);

    void _calculate_block_offsets(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                                  cuda_tool::DeviceBSRMatrix<T, N>&        to);


    // Doublet -> BCOO
    void convert(const cuda_tool::DeviceDoubletVector<T, N>& from,
                 cuda_tool::DeviceBCOOVector<T, N>&          to);

    void _radix_sort_indices_and_segments(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                          cuda_tool::DeviceBCOOVector<T, N>& to);

    void _make_unique_indices(const cuda_tool::DeviceDoubletVector<T, N>& from,
                              cuda_tool::DeviceBCOOVector<T, N>&          to);

    void _make_unique_segment_warp_reduction(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                             cuda_tool::DeviceBCOOVector<T, N>& to);


    template <typename U>
    void loose_resize(cuda_tool::DeviceBuffer<U>& buf, size_t new_size)
    {
        if(buf.capacity() < new_size)
            buf.reserve_discard(new_size * m_reserve_ratio);
        buf.resize_discard(new_size);
    }

    void ge2sym(cuda_tool::DeviceBCOOMatrix<T, N>& to);

    void ge2sym(cuda_tool::DeviceTripletMatrix<T, N>& to);

    void sym2ge(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                cuda_tool::DeviceBCOOMatrix<T, N>&       to);
};
}  // namespace uipc::backend::cuda

#include "details/matrix_converter.inl"
