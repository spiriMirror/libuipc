//#include <muda/cub/device/device_merge_sort.h>
//#include <muda/cub/device/device_radix_sort.h>
//#include <muda/cub/device/device_run_length_encode.h>
//#include <muda/cub/device/device_scan.h>
//#include <muda/cub/device/device_segmented_reduce.h>
//#include <muda/cub/device/device_reduce.h>
//#include <muda/launch.h>
//
//namespace muda::details
//{
//// using T = float;
//template <typename T>
//void MatrixFormatConverter<T, 1>::convert(const DeviceTripletMatrix<T, 1>& from,
//                                          DeviceBCOOMatrix<T, 1>&          to)
//{
//    to.reshape(from.rows(), from.cols());
//    to.resize_triplets(from.triplet_count());
//    if(to.triplet_count() == 0)
//        return;
//    merge_sort_indices_and_values(from, to);
//    make_unique_indices(from, to);
//    make_unique_values(from, to);
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::merge_sort_indices_and_values(
//    const DeviceTripletMatrix<T, 1>& from, DeviceBCOOMatrix<T, 1>& to)
//{
//    using namespace muda;
//
//    auto src_row_indices = from.row_indices();
//    auto src_col_indices = from.col_indices();
//    auto src_values      = from.values();
//
//    loose_resize(sort_index, src_row_indices.size());
//    loose_resize(ij_pairs, src_row_indices.size());
//
//    ParallelFor(256)
//        .kernel_name("set ij pairs")
//        .apply(src_row_indices.size(),
//               [row_indices = src_row_indices.cviewer().name("row_indices"),
//                col_indices = src_col_indices.cviewer().name("col_indices"),
//                ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
//               {
//                   ij_pairs(i).x = row_indices(i);
//                   ij_pairs(i).y = col_indices(i);
//               });
//
//    ParallelFor(256)
//        .kernel_name("iota")  //
//        .apply(src_row_indices.size(),
//               [sort_index = sort_index.viewer().name("sort_index")] __device__(int i) mutable
//               { sort_index(i) = i; });
//
//    DeviceMergeSort().SortPairs(ij_pairs.data(),
//                                sort_index.data(),
//                                ij_pairs.size(),
//                                [] __device__(const int2& a, const int2& b) {
//                                    return a.x < b.x || (a.x == b.x && a.y < b.y);
//                                });
//
//    // set ij_pairs back to row_indices and col_indices
//
//    auto dst_row_indices = to.row_indices();
//    auto dst_col_indices = to.col_indices();
//
//    ParallelFor(256)
//        .kernel_name("set col row indices")
//        .apply(dst_row_indices.size(),
//               [row_indices = dst_row_indices.viewer().name("row_indices"),
//                col_indices = dst_col_indices.viewer().name("col_indices"),
//                ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
//               {
//                   row_indices(i) = ij_pairs(i).x;
//                   col_indices(i) = ij_pairs(i).y;
//               });
//
//    // sort the block values
//
//    loose_resize(unique_values, from.m_values.size());
//
//    ParallelFor(256)
//        .kernel_name("set block values")
//        .apply(src_values.size(),
//               [src_values = src_values.cviewer().name("blocks"),
//                sort_index = sort_index.cviewer().name("sort_index"),
//                dst_values = unique_values.viewer().name("values")] __device__(int i) mutable
//               { dst_values(i) = src_values(sort_index(i)); });
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::make_unique_indices(const DeviceTripletMatrix<T, 1>& from,
//                                                      DeviceBCOOMatrix<T, 1>& to)
//{
//    using namespace muda;
//
//    auto& row_indices = to.m_row_indices;
//    auto& col_indices = to.m_col_indices;
//
//    loose_resize(unique_ij_pairs, ij_pairs.size());
//    loose_resize(unique_counts, ij_pairs.size());
//
//    DeviceRunLengthEncode().Encode(ij_pairs.data(),
//                                   unique_ij_pairs.data(),
//                                   unique_counts.data(),
//                                   count.data(),
//                                   ij_pairs.size());
//
//    int h_count = count;
//
//    unique_ij_pairs.resize(h_count);
//    unique_counts.resize(h_count);
//
//    loose_resize(offsets, unique_counts.size());
//
//    DeviceScan().ExclusiveSum(
//        unique_counts.data(), offsets.data(), unique_counts.size());
//
//
//    muda::ParallelFor(256)
//        .kernel_name("make unique indices")
//        .apply(unique_counts.size(),
//               [unique_ij_pairs = unique_ij_pairs.viewer().name("unique_ij_pairs"),
//                row_indices = row_indices.viewer().name("row_indices"),
//                col_indices = col_indices.viewer().name("col_indices")] __device__(int i) mutable
//               {
//                   row_indices(i) = unique_ij_pairs(i).x;
//                   col_indices(i) = unique_ij_pairs(i).y;
//               });
//
//    row_indices.resize(h_count);
//    col_indices.resize(h_count);
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::make_unique_values(const DeviceTripletMatrix<T, 1>& from,
//                                                     DeviceCOOMatrix<T>& to)
//{
//    using namespace muda;
//
//    auto& row_indices = to.m_row_indices;
//    auto& values      = to.m_values;
//    values.resize(row_indices.size());
//    // first we add the offsets to counts, to get the offset_ends
//
//    ParallelFor(256)
//        .kernel_name("calculate offset_ends")
//        .apply(unique_counts.size(),
//               [offset = offsets.cviewer().name("offset"),
//                counts = unique_counts.viewer().name("counts")] __device__(int i) mutable
//               { counts(i) += offset(i); });
//
//    auto& begin_offset = offsets;
//    auto& end_offset   = unique_counts;  // already contains the offset_ends
//
//    // then we do a segmented reduce to get the unique blocks
//    DeviceSegmentedReduce().Sum(unique_values.data(),
//                                values.data(),
//                                values.size(),
//                                offsets.data(),
//                                end_offset.data());
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::radix_sort_indices_and_blocks(
//    const DeviceTripletMatrix<T, 1>& from, DeviceBCOOMatrix<T, 1>& to)
//{
//    auto src_row_indices = from.row_indices();
//    auto src_col_indices = from.col_indices();
//    auto src_blocks      = from.values();
//
//    loose_resize(ij_hash_input, src_row_indices.size());
//    loose_resize(sort_index_input, src_row_indices.size());
//
//    loose_resize(ij_hash, src_row_indices.size());
//    loose_resize(sort_index, src_row_indices.size());
//    ij_pairs.resize(src_row_indices.size());
//
//
//    // hash ij
//    ParallelFor(256)
//        .kernel_name(__FUNCTION__)
//        .apply(src_row_indices.size(),
//               [row_indices = src_row_indices.cviewer().name("row_indices"),
//                col_indices = src_col_indices.cviewer().name("col_indices"),
//                ij_hash     = ij_hash_input.viewer().name("ij_hash"),
//                sort_index = sort_index_input.viewer().name("sort_index")] __device__(int i) mutable
//               {
//                   ij_hash(i) =
//                       (uint64_t{row_indices(i)} << 32) + uint64_t{col_indices(i)};
//                   sort_index(i) = i;
//               });
//
//    DeviceRadixSort().SortPairs(ij_hash_input.data(),
//                                ij_hash.data(),
//                                sort_index_input.data(),
//                                sort_index.data(),
//                                ij_hash.size());
//
//    // sort the block values
//
//    {
//        loose_resize(values_sorted, from.values().size());
//        ParallelFor(256)
//            .kernel_name(__FUNCTION__)
//            .apply(src_blocks.size(),
//                   [src_blocks = src_blocks.cviewer().name("blocks"),
//                    sort_index = sort_index.cviewer().name("sort_index"),
//                    dst_blocks = values_sorted.viewer().name("block_values")] __device__(int i) mutable
//                   { dst_blocks(i) = src_blocks(sort_index(i)); });
//    }
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::make_unique_indices_and_blocks(
//    const DeviceTripletMatrix<T, 1>& from, DeviceBCOOMatrix<T, 1>& to)
//{
//    // alias to reuse the memory
//    auto& unique_ij_hash = ij_hash_input;
//
//    muda::DeviceReduce().ReduceByKey(
//        ij_hash.data(),
//        unique_ij_hash.data(),
//        values_sorted.data(),
//        to.values().data(),
//        count.data(),
//        [] CUB_RUNTIME_FUNCTION(const T& l, const T& r) -> T { return l + r; },
//        ij_hash.size());
//
//    int h_count = count;
//
//    to.resize_triplets(h_count);
//
//    // set ij_hash back to row_indices and col_indices
//    ParallelFor()
//        .kernel_name("set col row indices")
//        .apply(to.row_indices().size(),
//               [ij_hash     = unique_ij_hash.viewer().name("ij_hash"),
//                row_indices = to.row_indices().viewer().name("row_indices"),
//                col_indices = to.col_indices().viewer().name("col_indices")] __device__(int i) mutable
//               {
//                   auto hash      = ij_hash(i);
//                   auto row_index = int{hash >> 32};
//                   auto col_index = int{hash & 0xFFFFFFFF};
//                   row_indices(i) = row_index;
//                   col_indices(i) = col_index;
//               });
//}
//
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::convert(const DeviceCOOMatrix<T>& from,
//                                          DeviceDenseMatrix<T>&     to,
//                                          bool clear_dense_matrix)
//{
//    using namespace muda;
//    auto size = from.rows();
//    to.reshape(size, size);
//
//    if(clear_dense_matrix)
//        to.fill(0);
//
//    ParallelFor(256)
//        .kernel_name(__FUNCTION__)
//        .apply(from.values().size(),
//               [values = from.cviewer().name("src_sparse_matrix"),
//                dst = to.viewer().name("dst_dense_matrix")] __device__(int i) mutable
//               {
//                   auto value = values(i);
//                   auto row   = value.row_index;
//                   auto col   = value.col_index;
//                   dst(row, col) += value.value;
//               });
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::convert(const DeviceCOOMatrix<T>& from,
//                                          DeviceCSRMatrix<T>&       to)
//{
//    calculate_block_offsets(from, to);
//    to.m_col_indices = from.m_col_indices;
//    to.m_values      = from.m_values;
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::convert(DeviceCOOMatrix<T>&& from, DeviceCSRMatrix<T>& to)
//{
//    calculate_block_offsets(from, to);
//    to.m_col_indices = std::move(from.m_col_indices);
//    to.m_values      = std::move(from.m_values);
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::calculate_block_offsets(const DeviceCOOMatrix<T>& from,
//                                                          DeviceCSRMatrix<T>& to)
//{
//    using namespace muda;
//    to.reshape(from.rows(), from.cols());
//
//    auto& dst_row_offsets = to.m_row_offsets;
//
//    // alias the offsets to the col_counts_per_row(reuse)
//    auto& col_counts_per_row = offsets;
//    col_counts_per_row.resize(to.m_row_offsets.size());
//    col_counts_per_row.fill(0);
//
//    loose_resize(unique_indices, from.non_zeros());
//    loose_resize(unique_counts, from.non_zeros());
//
//    // run length encode the row
//    DeviceRunLengthEncode().Encode(from.m_row_indices.data(),
//                                   unique_indices.data(),
//                                   unique_counts.data(),
//                                   count.data(),
//                                   from.non_zeros());
//    int h_count = count;
//
//    unique_indices.resize(h_count);
//    unique_counts.resize(h_count);
//
//    ParallelFor(256)
//        .kernel_name(__FUNCTION__)
//        .apply(unique_counts.size(),
//               [unique_indices     = unique_indices.cviewer().name("offset"),
//                counts             = unique_counts.viewer().name("counts"),
//                col_counts_per_row = col_counts_per_row.viewer().name(
//                    "col_counts_per_row")] __device__(int i) mutable
//               {
//                   auto row                = unique_indices(i);
//                   col_counts_per_row(row) = counts(i);
//               });
//
//    // calculate the offsets
//    DeviceScan().ExclusiveSum(col_counts_per_row.data(),
//                              dst_row_offsets.data(),
//                              col_counts_per_row.size());
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::convert(const DeviceDoubletVector<T, 1>& from,
//                                          DeviceCOOVector<T>&              to)
//{
//    to.reshape(from.size());
//    to.resize_doublet(from.doublet_count());
//
//    merge_sort_indices_and_values(from, to);
//    make_unique_indices(from, to);
//    make_unique_values(from, to);
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::merge_sort_indices_and_values(
//    const DeviceDoubletVector<T, 1>& from, DeviceCOOVector<T>& to)
//{
//    using namespace muda;
//
//    auto& indices = sort_index;
//    auto& values  = temp_values;
//
//    indices = from.m_indices;
//    values  = from.m_values;
//
//    DeviceMergeSort().SortPairs(indices.data(),
//                                values.data(),
//                                indices.size(),
//                                [] __device__(const int& a, const int& b)
//                                { return a < b; });
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::make_unique_indices(const DeviceDoubletVector<T, 1>& from,
//                                                      DeviceCOOVector<T>& to)
//{
//    using namespace muda;
//
//    auto& indices = to.m_indices;
//    auto& values  = to.m_values;
//
//    auto& unique_indices = to.m_indices;
//    unique_indices.resize(indices.size());
//    loose_resize(unique_counts, indices.size());
//
//    DeviceRunLengthEncode().Encode(indices.data(),
//                                   unique_indices.data(),
//                                   unique_counts.data(),
//                                   count.data(),
//                                   indices.size());
//
//    int h_count = count;
//
//    unique_indices.resize(h_count);
//    unique_counts.resize(h_count);
//    loose_resize(offsets, unique_counts.size());
//
//    DeviceScan().ExclusiveSum(
//        unique_counts.data(), offsets.data(), unique_counts.size());
//
//    // calculate the offset_ends, and set to the unique_counts
//    auto& begin_offset = offsets;
//    auto& end_offset   = unique_counts;
//
//    ParallelFor(256)
//        .kernel_name("calculate offset_ends")
//        .apply(unique_counts.size(),
//               [offset = offsets.cviewer().name("offset"),
//                counts = unique_counts.viewer().name("counts")] __device__(int i) mutable
//               { counts(i) += offset(i); });
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::make_unique_values(const DeviceDoubletVector<T, 1>& from,
//                                                     DeviceCOOVector<T>& to)
//{
//    using namespace muda;
//
//    auto& begin_offset = offsets;
//    auto& end_offset   = unique_counts;
//
//    auto& unique_values = to.m_values;
//    unique_values.resize(unique_indices.size());
//
//    DeviceSegmentedReduce().Sum(temp_values.data(),
//                                unique_values.data(),
//                                unique_values.size(),
//                                begin_offset.data(),
//                                end_offset.data());
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::convert(const DeviceCOOVector<T>& from,
//                                          DeviceDenseVector<T>&     to,
//                                          bool clear_dense_vector)
//{
//    to.resize(from.size());
//    set_unique_values_to_dense_vector(from, to, clear_dense_vector);
//}
//
//template <typename T>
//void MatrixFormatConverter<T, 1>::set_unique_values_to_dense_vector(
//    const DeviceDoubletVector<T, 1>& from, DeviceDenseVector<T>& to, bool clear_dense_vector)
//{
//    using namespace muda;
//
//    if(clear_dense_vector)
//        to.fill(0);
//
//    auto& unique_values  = from.m_values;
//    auto& unique_indices = from.m_indices;
//
//    ParallelFor(256)
//        .kernel_name("set unique values to dense vector")
//        .apply(unique_values.size(),
//               [unique_values = unique_values.cviewer().name("unique_values"),
//                unique_indices = unique_indices.cviewer().name("unique_indices"),
//                dst = to.viewer().name("dst_dense_vector")] __device__(int i) mutable
//               {
//                   auto index = unique_indices(i);
//                   dst(index) += unique_values(i);
//               });
//}
//
//// using T = float;
//template <typename T>
//void MatrixFormatConverter<T, 1>::convert(const DeviceDoubletVector<T, 1>& from,
//                                          DeviceDenseVector<T>&            to,
//                                          bool clear_dense_vector)
//{
//    using namespace muda;
//
//    to.resize(from.count());
//
//    if(clear_dense_vector)
//        to.fill(0);
//
//    ParallelFor(256)
//        .kernel_name(__FUNCTION__)
//        .apply(from.doublet_count(),
//               [src = from.viewer().name("src_sparse_vector"),
//                dst = to.viewer().name("dst_dense_vector")] __device__(int i) mutable
//               {
//                   auto&& [index, value] = src(i);
//                   dst.segment<1>(index).atomic_add(value);
//               });
//}
//}  // namespace muda::details