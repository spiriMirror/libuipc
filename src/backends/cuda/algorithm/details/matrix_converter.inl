#include <cuda_tool/cuda_tool.h>
#include <cub/warp/warp_reduce.cuh>
#include <cuda_tool/cuda_tool.h>
#include <uipc/common/timer.h>
#include <algorithm/fast_segmental_reduce.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
template <typename T, int N>
void MatrixConverter<T, N>::convert(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                    cuda_tool::DeviceBCOOMatrix<T, N>&          to)
{
    to.reshape(from.rows(), from.cols());
    to.resize_triplets(from.triplet_count());


    if(to.triplet_count() == 0)
        return;

    _radix_sort_indices_and_blocks(from, to);

    _make_unique_indices(from, to);
    _make_unique_block_warp_reduction(from, to);
}

template <typename T, int N>
void MatrixConverter<T, N>::_radix_sort_indices_and_blocks(
    const cuda_tool::DeviceTripletMatrix<T, N>& from, cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    auto src_row_indices = from.row_indices();
    auto src_col_indices = from.col_indices();
    auto src_blocks      = from.values();

    loose_resize(ij_hash_input, src_row_indices.size());
    loose_resize(sort_index_input, src_row_indices.size());

    loose_resize(ij_hash, src_row_indices.size());
    loose_resize(sort_index, src_row_indices.size());
    ij_pairs.resize(src_row_indices.size());


    // hash ij
    ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(src_row_indices.size(),
               [row_indices = src_row_indices.cviewer(),
                col_indices = src_col_indices.cviewer(),
                ij_hash     = ij_hash_input.viewer(),
                sort_index = sort_index_input.viewer()] __device__(int i) mutable
               {
                   ij_hash(i) = (static_cast<uint64_t>(row_indices(i)) << 32)
                                + static_cast<uint64_t>(col_indices(i));
                   sort_index(i) = i;
               });

    DeviceRadixSort().SortPairs(ij_hash_input.data(),
                                ij_hash.data(),
                                sort_index_input.data(),
                                sort_index.data(),
                                ij_hash.size());

    // set ij_hash back to row_indices and col_indices

    auto dst_row_indices = to.row_indices();
    auto dst_col_indices = to.col_indices();

    ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(dst_row_indices.size(),
               [ij_hash = ij_hash.viewer(),
                ij_pairs = ij_pairs.viewer()] __device__(int i) mutable
               {
                   auto hash      = ij_hash(i);
                   auto row_index = static_cast<int>(hash >> 32);
                   auto col_index = static_cast<int>(hash & 0xFFFFFFFF);
                   ij_pairs(i).x  = row_index;
                   ij_pairs(i).y  = col_index;
               });

    // sort the block values

    {
        loose_resize(blocks_sorted, from.values().size());
        ParallelFor(256)
            .file_line(__FILE__, __LINE__)
            .apply(src_blocks.size(),
                   [src_blocks = src_blocks.cviewer(),
                    sort_index = sort_index.cviewer(),
                    dst_blocks = blocks_sorted.viewer()] __device__(int i) mutable
                   { dst_blocks(i) = src_blocks(sort_index(i)); });
    }
}

template <typename T, int N>
void MatrixConverter<T, N>::_radix_sort_indices_and_blocks(cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    auto src_row_indices = to.row_indices();
    auto src_col_indices = to.col_indices();
    auto src_blocks      = to.values();

    loose_resize(ij_hash_input, src_row_indices.size());
    loose_resize(sort_index_input, src_row_indices.size());

    loose_resize(ij_hash, src_row_indices.size());
    loose_resize(sort_index, src_row_indices.size());
    ij_pairs.resize(src_row_indices.size());


    // hash ij
    ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(src_row_indices.size(),
               [row_indices = src_row_indices.cviewer(),
                col_indices = src_col_indices.cviewer(),
                ij_hash     = ij_hash_input.viewer(),
                sort_index = sort_index_input.viewer()] __device__(int i) mutable
               {
                   ij_hash(i) =
                       (uint64_t{row_indices(i)} << 32) + uint64_t{col_indices(i)};
                   sort_index(i) = i;
               });

    DeviceRadixSort().SortPairs(ij_hash_input.data(),
                                ij_hash.data(),
                                sort_index_input.data(),
                                sort_index.data(),
                                ij_hash.size());

    // set ij_hash back to row_indices and col_indices

    auto dst_row_indices = to.row_indices();
    auto dst_col_indices = to.col_indices();

    ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(dst_row_indices.size(),
               [ij_hash = ij_hash.viewer(),
                ij_pairs = ij_pairs.viewer()] __device__(int i) mutable
               {
                   auto hash      = ij_hash(i);
                   auto row_index = int{hash >> 32};
                   auto col_index = int{hash & 0xFFFFFFFF};
                   ij_pairs(i).x  = row_index;
                   ij_pairs(i).y  = col_index;
               });

    // sort the block values

    {
        loose_resize(blocks_sorted, to.values().size());
        ParallelFor(256)
            .file_line(__FILE__, __LINE__)
            .apply(src_blocks.size(),
                   [src_blocks = src_blocks.cviewer(),
                    sort_index = sort_index.cviewer(),
                    ij_pairs   = ij_pairs.cviewer(),
                    dst_row    = to.row_indices().viewer(),
                    dst_col    = to.col_indices().viewer(),

                    dst_blocks = blocks_sorted.viewer()] __device__(int i) mutable
                   {
                       dst_blocks(i) = src_blocks(sort_index(i));
                       dst_row(i)    = ij_pairs(i).x;
                       dst_col(i)    = ij_pairs(i).y;
                   });

        to.values().copy_from(blocks_sorted);
    }
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_indices(const cuda_tool::DeviceTripletMatrix<T, N>& from,
                                                 cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    auto row_indices = to.row_indices();
    auto col_indices = to.col_indices();

    loose_resize(unique_ij_pairs, ij_pairs.size());
    loose_resize(unique_counts, ij_pairs.size());


    DeviceRunLengthEncode().Encode(ij_pairs.data(),
                                   unique_ij_pairs.data(),
                                   unique_counts.data(),
                                   count.data(),
                                   ij_pairs.size());

    int h_count = count;

    unique_ij_pairs.resize(h_count);
    unique_counts.resize(h_count);

    offsets.resize(unique_counts.size());

    DeviceScan().ExclusiveSum(
        unique_counts.data(), offsets.data(), unique_counts.size());


    cuda_tool::ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(unique_counts.size(),
               [unique_ij_pairs = unique_ij_pairs.viewer(),
                row_indices = row_indices.viewer(),
                col_indices = col_indices.viewer()] __device__(int i) mutable
               {
                   row_indices(i) = unique_ij_pairs(i).x;
                   col_indices(i) = unique_ij_pairs(i).y;
               });

    to.resize_triplets(h_count);
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_block_warp_reduction(
    const cuda_tool::DeviceTripletMatrix<T, N>& from, cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    loose_resize(sorted_partition_input, ij_pairs.size());
    loose_resize(sorted_partition_output, ij_pairs.size());


    BufferLaunch().fill<int>(sorted_partition_input, 0);

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(unique_counts.size(),
               [sorted_partition = sorted_partition_input.viewer(),
                unique_counts = unique_counts.viewer(),
                offsets = offsets.viewer()] __device__(int i) mutable
               {
                   auto offset = offsets(i);
                   auto count  = unique_counts(i);

                   sorted_partition(offset + count - 1) = 1;
               });

    // scatter
    DeviceScan().ExclusiveSum(sorted_partition_input.data(),
                              sorted_partition_output.data(),
                              sorted_partition_input.size());

    auto blocks = to.values();

    FastSegmentalReduce<>()
        .file_line(__FILE__, __LINE__)
        .reduce(std::as_const(sorted_partition_output).view(),
                std::as_const(blocks_sorted).view(),
                blocks);
}

template <typename T, int N>
void MatrixConverter<T, N>::convert(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                                    cuda_tool::DeviceBSRMatrix<T, N>&        to)
{
    // calculate the row offsets
    _calculate_block_offsets(from, to);

    to.resize(from.non_zeros());

    auto vals        = to.values();
    auto col_indices = to.col_indices();

    vals.copy_from(from.values());  // BCOO and BSR have the same block values
    col_indices.copy_from(from.col_indices());  // BCOO and BSR have the same block col indices
}

template <typename T, int N>
void MatrixConverter<T, N>::_calculate_block_offsets(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                                                     cuda_tool::DeviceBSRMatrix<T, N>& to)
{
    //Timer timer{__FUNCTION__};

    using namespace cuda_tool;
    to.reshape(from.rows(), from.cols());


    auto dst_row_offsets = to.row_offsets();

    col_counts_per_row.resize(dst_row_offsets.size());
    col_counts_per_row.fill(0);

    unique_indices.resize(from.non_zeros());
    unique_counts.resize(from.non_zeros());


    // run length encode the row
    DeviceRunLengthEncode().Encode(from.row_indices().data(),
                                   unique_indices.data(),
                                   unique_counts.data(),
                                   count.data(),
                                   from.non_zeros());
    int h_count = count;

    unique_indices.resize(h_count);
    unique_counts.resize(h_count);

    ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(unique_counts.size(),
               [unique_indices     = unique_indices.cviewer(),
                counts             = unique_counts.viewer(),
                col_counts_per_row = col_counts_per_row.viewer()] __device__(int i) mutable
               {
                   auto row                = unique_indices(i);
                   col_counts_per_row(row) = counts(i);
               });

    // calculate the offsets
    DeviceScan().ExclusiveSum(col_counts_per_row.data(),
                              dst_row_offsets.data(),
                              col_counts_per_row.size());
}

//using T         = Float;
//constexpr int N = 3;

template <typename T, int N>
void MatrixConverter<T, N>::convert(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                    cuda_tool::DeviceBCOOVector<T, N>&          to)
{
    to.reshape(from.count());
    to.resize_doublets(from.doublet_count());

    if(to.doublet_count() == 0)
        return;

    _radix_sort_indices_and_segments(from, to);
    _make_unique_indices(from, to);
    _make_unique_segment_warp_reduction(from, to);
}

template <typename T, int N>
void MatrixConverter<T, N>::_radix_sort_indices_and_segments(
    const cuda_tool::DeviceDoubletVector<T, N>& from, cuda_tool::DeviceBCOOVector<T, N>& to)
{
    using namespace cuda_tool;

    auto src_indices  = from.indices();
    auto src_segments = from.values();

    loose_resize(indices_sorted, src_indices.size());
    loose_resize(segments_sorted, src_segments.size());

    DeviceRadixSort().SortPairs(src_indices.data(),
                                indices_sorted.data(),
                                src_segments.data(),
                                segments_sorted.data(),
                                src_indices.size());
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_indices(const cuda_tool::DeviceDoubletVector<T, N>& from,
                                                 cuda_tool::DeviceBCOOVector<T, N>& to)
{
    using namespace cuda_tool;

    auto dst_indices  = to.indices();
    auto dst_segments = to.values();
    loose_resize(unique_indices, indices_sorted.size());
    loose_resize(unique_counts, indices_sorted.size());

    DeviceRunLengthEncode().Encode(indices_sorted.data(),
                                   unique_indices.data(),
                                   unique_counts.data(),
                                   count.data(),
                                   indices_sorted.size());

    int h_count = count;

    unique_indices.resize(h_count);
    unique_counts.resize(h_count);

    offsets.resize(unique_counts.size());

    DeviceScan().ExclusiveSum(
        unique_counts.data(), offsets.data(), unique_counts.size());

    cuda_tool::ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(unique_counts.size(),
               [unique_indices = unique_indices.viewer(),
                dst_indices = dst_indices.viewer()] __device__(int i) mutable
               { dst_indices(i) = unique_indices(i); });

    to.resize_doublets(h_count);
}

template <typename T, int N>
void MatrixConverter<T, N>::_make_unique_segment_warp_reduction(
    const cuda_tool::DeviceDoubletVector<T, N>& from, cuda_tool::DeviceBCOOVector<T, N>& to)
{
    using namespace cuda_tool;

    loose_resize(sorted_partition_input, indices_sorted.size());
    loose_resize(sorted_partition_output, indices_sorted.size());

    BufferLaunch().fill<int>(sorted_partition_input, 0);

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(unique_counts.size(),
               [sorted_partition = sorted_partition_input.viewer(),
                unique_counts = unique_counts.viewer(),
                offsets = offsets.viewer()] __device__(int i) mutable
               {
                   auto offset = offsets(i);
                   auto count  = unique_counts(i);

                   sorted_partition(offset + count - 1) = 1;
               });

    // scatter
    DeviceScan().ExclusiveSum(sorted_partition_input.data(),
                              sorted_partition_output.data(),
                              sorted_partition_input.size());

    auto segments = to.values();

    FastSegmentalReduce<64, 32>()
        .file_line(__FILE__, __LINE__)
        .reduce(std::as_const(sorted_partition_output).view(),
                std::as_const(segments_sorted).view(),
                segments);
}

template <typename T, int N>
void MatrixConverter<T, N>::ge2sym(cuda_tool::DeviceBCOOMatrix<T, N>& to)
{
    using namespace cuda_tool;

    // alias to reuse the memory
    auto& counts     = unique_counts;
    auto& block_temp = blocks_sorted;

    loose_resize(counts, to.non_zeros());
    loose_resize(offsets, to.non_zeros());
    loose_resize(ij_pairs, to.non_zeros());
    loose_resize(block_temp, to.values().size());

    // 0. find the upper triangular part (where i <= j)
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(to.non_zeros(),
               [row_indices = to.row_indices().cviewer(),
                col_indices = to.col_indices().cviewer(),
                ij_pairs    = ij_pairs.viewer(),
                blocks      = to.values().cviewer(),
                block_temp  = block_temp.viewer(),
                counts = counts.viewer()] __device__(int i) mutable
               {
                   counts(i)     = row_indices(i) <= col_indices(i) ? 1 : 0;
                   ij_pairs(i).x = row_indices(i);
                   ij_pairs(i).y = col_indices(i);
                   block_temp(i) = blocks(i);
               });

    // exclusive sum
    DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());

    // set the values
    auto dst_block = to.values();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(dst_block.size(),
               [dst_blocks  = dst_block.viewer(),
                src_blocks  = block_temp.cviewer(),
                ij_pairs    = ij_pairs.cviewer(),
                row_indices = to.row_indices().viewer(),
                col_indices = to.col_indices().viewer(),
                counts      = counts.cviewer(),
                offsets     = offsets.cviewer(),
                total_count = count.viewer()] __device__(int i) mutable
               {
                   auto count  = counts(i);
                   auto offset = offsets(i);

                   if(count != 0)
                   {
                       dst_blocks(offset)  = src_blocks(i);
                       auto ij             = ij_pairs(i);
                       row_indices(offset) = ij.x;
                       col_indices(offset) = ij.y;
                   }

                   if(i == offsets.total_size() - 1)
                   {
                       total_count = offsets(i) + counts(i);
                   }
               });

    int h_total_count = count;

    to.resize_triplets(h_total_count);
}

template <typename T, int N>
void MatrixConverter<T, N>::ge2sym(cuda_tool::DeviceTripletMatrix<T, N>& to)
{
    using namespace cuda_tool;

    // alias to reuse the memory
    auto& counts     = unique_counts;
    auto& block_temp = blocks_sorted;

    loose_resize(counts, to.triplet_count());
    loose_resize(offsets, to.triplet_count());
    loose_resize(ij_pairs, to.triplet_count());
    loose_resize(block_temp, to.values().size());

    // 0. find the upper triangular part (where i <= j)
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(to.triplet_count(),
               [row_indices = to.row_indices().cviewer(),
                col_indices = to.col_indices().cviewer(),
                ij_pairs    = ij_pairs.viewer(),
                blocks      = to.values().cviewer(),
                block_temp  = block_temp.viewer(),
                counts = counts.viewer()] __device__(int i) mutable
               {
                   counts(i)     = row_indices(i) <= col_indices(i) ? 1 : 0;
                   ij_pairs(i).x = row_indices(i);
                   ij_pairs(i).y = col_indices(i);
                   block_temp(i) = blocks(i);
               });

    // exclusive sum
    DeviceScan().ExclusiveSum(counts.data(), offsets.data(), counts.size());

    // set the values
    auto dst_block = to.values();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(dst_block.size(),
               [dst_blocks  = dst_block.viewer(),
                src_blocks  = block_temp.cviewer(),
                ij_pairs    = ij_pairs.cviewer(),
                row_indices = to.row_indices().viewer(),
                col_indices = to.col_indices().viewer(),
                counts      = counts.cviewer(),
                offsets     = offsets.cviewer(),
                total_count = count.viewer()] __device__(int i) mutable
               {
                   auto count  = counts(i);
                   auto offset = offsets(i);

                   if(count != 0)
                   {
                       dst_blocks(offset)  = src_blocks(i);
                       auto ij             = ij_pairs(i);
                       row_indices(offset) = ij.x;
                       col_indices(offset) = ij.y;
                   }

                   if(i == offsets.total_size() - 1)
                   {
                       total_count = offsets(i) + counts(i);
                   }
               });

    int h_total_count = count;

    to.resize_triplets(h_total_count);
}


template <typename T, int N>
void MatrixConverter<T, N>::sym2ge(const cuda_tool::DeviceBCOOMatrix<T, N>& from,
                                   cuda_tool::DeviceBCOOMatrix<T, N>&       to)
{
    using namespace cuda_tool;

    auto sym_size = from.non_zeros();

    // alias to reuse the memory
    auto& flags                 = offsets;
    auto& partitioned           = blocks_sorted;
    auto& partition_index_input = sort_index_input;
    auto& partition_index       = sort_index;
    auto& selected_count        = count;
    auto  diag_count            = from.rows();


    loose_resize(flags, sym_size);
    loose_resize(partitioned, sym_size);
    loose_resize(partition_index, sym_size);

    // setup select flag
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(sym_size,
               [flags       = flags.viewer(),
                row_indices = from.row_indices().cviewer(),
                col_indices = from.col_indices().cviewer(),
                partition_index = partition_index_input.viewer()] __device__(int i) mutable
               {
                   flags(i) = (row_indices(i) == col_indices(i)) ? 1 : 0;
                   partition_index(i) = i;
               });


    cuda_tool::DevicePartition().Flagged(partition_index_input.data(),
                                    flags.data(),
                                    partition_index.data(),
                                    selected_count.data(),
                                    sym_size);


    auto general_bcoo_size = 2 * (sym_size - diag_count) + diag_count;

    to.resize(from.rows(), from.cols(), general_bcoo_size);

    // copy blocks and ij
    // in this sequence:
    // [ Diag | Upper | Lower ]
    //
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(sym_size,
               [to   = to.viewer(),
                from = from.cviewer(),
                partition_index = partition_index.cviewer(),
                diag_count = diag_count,
                sym_size   = sym_size] __device__(int i) mutable
               {
                   auto index = partition_index(i);
                   auto f     = from(index);
                   // diag + upper
                   to(i).write(f.row_index, f.col_index, f.value);
                   if(i >= diag_count)
                   {
                       // lower
                       to(i + sym_size - diag_count)
                           .write(f.col_index, f.row_index, f.value.transpose());
                   }
               });

    _radix_sort_indices_and_blocks(to);
}
}  // namespace uipc::backend::cuda