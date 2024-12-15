#include <affine_body/matrix_converter.h>
#include <muda/cub/device/device_merge_sort.h>
#include <muda/cub/device/device_run_length_encode.h>
#include <muda/cub/device/device_scan.h>
#include <muda/cub/device/device_segmented_reduce.h>
#include <muda/cub/device/device_radix_sort.h>
#include <muda/cub/device/device_select.h>
#include <cub/warp/warp_reduce.cuh>
#include <muda/ext/eigen/atomic.h>
#include <uipc/common/timer.h>
#include <algorithm/fast_segmental_reduce.h>
#include <muda/cub/device/device_reduce.h>
#include <muda/cub/device/device_partition.h>

// for encode run length usage
MUDA_GENERIC constexpr bool operator==(const int2& a, const int2& b)
{
    return a.x == b.x && a.y == b.y;
}

namespace uipc::backend::cuda
{
void ABDMatrixConverter::convert(const muda::DeviceTripletMatrix<T, N>& from,
                                 muda::DeviceBCOOMatrix<T, N>&          to)
{
    m_impl.convert(from, to);
}

void ABDMatrixConverter::Impl::convert(const muda::DeviceTripletMatrix<T, N>& from,
                                       muda::DeviceBCOOMatrix<T, N>& to)
{
    to.reshape(from.rows(), from.cols());
    to.resize_triplets(from.triplet_count());


    if(to.triplet_count() == 0)
        return;

    _radix_sort_indices_and_blocks(from, to);
    _make_unique_indices(from, to);
    _make_unique_block_warp_reduction(from, to);
}


void ABDMatrixConverter::Impl::_radix_sort_indices_and_blocks(
    const muda::DeviceTripletMatrix<T, N>& from, muda::DeviceBCOOMatrix<T, N>& to)
{
    using namespace muda;

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
        .kernel_name(__FUNCTION__)
        .apply(src_row_indices.size(),
               [row_indices = src_row_indices.cviewer().name("row_indices"),
                col_indices = src_col_indices.cviewer().name("col_indices"),
                ij_hash     = ij_hash_input.viewer().name("ij_hash"),
                sort_index = sort_index_input.viewer().name("sort_index")] __device__(int i) mutable
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
        .kernel_name("set col row indices")
        .apply(dst_row_indices.size(),
               [ij_hash = ij_hash.viewer().name("ij_hash"),
                ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
               {
                   auto hash      = ij_hash(i);
                   auto row_index = int{hash >> 32};
                   auto col_index = int{hash & 0xFFFFFFFF};
                   ij_pairs(i).x  = row_index;
                   ij_pairs(i).y  = col_index;
               });

    // sort the block values

    {
        Timer timer("set block values");
        loose_resize(blocks_sorted, from.values().size());
        ParallelFor(256)
            .kernel_name(__FUNCTION__)
            .apply(src_blocks.size(),
                   [src_blocks = src_blocks.cviewer().name("blocks"),
                    sort_index = sort_index.cviewer().name("sort_index"),
                    dst_blocks = blocks_sorted.viewer().name("values")] __device__(int i) mutable
                   { dst_blocks(i) = src_blocks(sort_index(i)); });
    }
}

void ABDMatrixConverter::Impl::_make_unique_indices(const muda::DeviceTripletMatrix<T, N>& from,
                                                    muda::DeviceBCOOMatrix<T, N>& to)
{
    using namespace muda;

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

    offsets.resize(unique_counts.size() + 1);  // +1 for the last offset_end

    DeviceScan().ExclusiveSum(
        unique_counts.data(), offsets.data(), unique_counts.size());


    muda::ParallelFor(256)
        .kernel_name(__FUNCTION__)
        .apply(unique_counts.size(),
               [unique_ij_pairs = unique_ij_pairs.viewer().name("unique_ij_pairs"),
                row_indices = row_indices.viewer().name("row_indices"),
                col_indices = col_indices.viewer().name("col_indices")] __device__(int i) mutable
               {
                   row_indices(i) = unique_ij_pairs(i).x;
                   col_indices(i) = unique_ij_pairs(i).y;
               });

    to.resize_triplets(h_count);
}

void ABDMatrixConverter::Impl::_make_unique_block_warp_reduction(
    const muda::DeviceTripletMatrix<T, N>& from, muda::DeviceBCOOMatrix<T, N>& to)
{
    using namespace muda;

    loose_resize(sorted_partition_input, ij_pairs.size());
    loose_resize(sorted_partition_output, ij_pairs.size());


    BufferLaunch().fill<int>(sorted_partition_input, 0);

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(unique_counts.size(),
               [sorted_partition = sorted_partition_input.viewer().name("sorted_partition"),
                unique_counts = unique_counts.viewer().name("unique_counts"),
                offsets = offsets.viewer().name("offsets")] __device__(int i) mutable
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


    FastSegmentalReduce()
        .kernel_name(__FUNCTION__)
        .reduce(std::as_const(sorted_partition_output).view(),
                std::as_const(blocks_sorted).view(),
                blocks);
}
}  // namespace uipc::backend::cuda