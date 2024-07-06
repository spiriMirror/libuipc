//#pragma once
//#include <type_define.h>
//#include <muda/buffer/device_buffer.h>
//#include <muda/ext/linear_system/matrix_format_converter.h>
//
//namespace uipc::backend::cuda
//{
//class MatrixConverter
//{
//    using T                = Float;
//    constexpr static int N = 3;
//
//    using BlockMatrix   = muda::DeviceTripletMatrix<T, N>::BlockMatrix;
//    using SegmentVector = muda::DeviceDoubletVector<T, N>::SegmentVector;
//
//    muda::DeviceBuffer<int> col_counts_per_row;
//    muda::DeviceBuffer<int> unique_indices;
//    muda::DeviceBuffer<int> unique_counts;
//    muda::DeviceVar<int>    count;
//
//    muda::DeviceBuffer<int> sort_index;
//    muda::DeviceBuffer<int> sort_index_input;
//
//    muda::DeviceBuffer<int> col_tmp;
//    muda::DeviceBuffer<int> row_tmp;
//
//    muda::DeviceBCOOMatrix<T, N> temp_bcoo_matrix;
//    muda::DeviceBCOOVector<T, N> temp_bcoo_vector;
//
//    muda::DeviceBuffer<int> offsets;
//
//    muda::DeviceBuffer<int2>     ij_pairs;
//    muda::DeviceBuffer<uint64_t> ij_hash;
//    muda::DeviceBuffer<uint64_t> ij_hash_input;
//    muda::DeviceBuffer<int2>     unique_ij_pairs;
//
//    muda::DeviceBuffer<BlockMatrix>   blocks_sorted;
//    muda::DeviceBuffer<BlockMatrix>   diag_blocks;
//    muda::DeviceBuffer<SegmentVector> unique_segments;
//    muda::DeviceBuffer<SegmentVector> temp_segments;
//
//    muda::DeviceBuffer<T> unique_values;
//
//    Float reserve_ratio = 1.5;
//
//    muda::DeviceBuffer<int> warp_count_per_unique_block;
//    muda::DeviceBuffer<int> warp_offset_per_unique_block;
//    muda::DeviceBuffer<int> warp_id_to_unique_block_id;
//    muda::DeviceBuffer<int> warp_id_in_unique_block;
//    muda::DeviceBuffer<int> sorted_partition_input;
//    muda::DeviceBuffer<int> sorted_partition_output;
//
//  public:
//    // Triplet -> BCOO
//    void convert(const muda::DeviceTripletMatrix<T, N>& from,
//                 muda::DeviceBCOOMatrix<T, N>&          to);
//
//    void _merge_sort_indices_and_blocks(const muda::DeviceTripletMatrix<T, N>& from,
//                                        muda::DeviceBCOOMatrix<T, N>& to);
//
//    void _radix_sort_indices_and_blocks(const muda::DeviceTripletMatrix<T, N>& from,
//                                        muda::DeviceBCOOMatrix<T, N>& to);
//
//    void _radix_sort_indices_and_blocks(muda::DeviceBCOOMatrix<T, N>& to);
//
//    void _make_unique_indices(const muda::DeviceTripletMatrix<T, N>& from,
//                              muda::DeviceBCOOMatrix<T, N>&          to);
//
//    void _make_unique_indices_and_blocks(const muda::DeviceTripletMatrix<T, N>& from,
//                                         muda::DeviceBCOOMatrix<T, N>& to);
//
//    void _make_unique_blocks_task_based(const muda::DeviceTripletMatrix<T, N>& from,
//                                        muda::DeviceBCOOMatrix<T, N>& to);
//
//    void _make_unique_block_warp_reduction(const muda::DeviceTripletMatrix<T, N>& from,
//                                           muda::DeviceBCOOMatrix<T, N>& to);
//
//    void _make_unique_blocks_naive(const muda::DeviceTripletMatrix<T, N>& from,
//                                   muda::DeviceBCOOMatrix<T, N>&          to);
//
//    // BCOO -> BSR
//    void convert(const muda::DeviceBCOOMatrix<T, N>& from,
//                 muda::DeviceBSRMatrix<T, N>&        to);
//
//    void _calculate_block_offsets(const muda::DeviceBCOOMatrix<T, N>& from,
//                                  muda::DeviceBSRMatrix<T, N>&        to);
//
//    template <typename T>
//    void loose_resize(muda::DeviceBuffer<T>& buf, size_t new_size)
//    {
//        if(buf.capacity() < new_size)
//            buf.reserve(new_size * reserve_ratio);
//        buf.resize(new_size);
//    }
//
//    void ge2sym(muda::DeviceBCOOMatrix<T, N>& to);
//
//    void sym2ge(const muda::DeviceBCOOMatrix<T, N>& from,
//                muda::DeviceBCOOMatrix<T, N>&       to);
//};
//}  // namespace uipc::backend::cuda
