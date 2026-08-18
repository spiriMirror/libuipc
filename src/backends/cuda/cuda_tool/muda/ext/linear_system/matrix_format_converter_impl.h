#pragma once
#include <muda/ext/linear_system/linear_system_handles.h>
#include <muda/ext/linear_system/device_dense_matrix.h>
#include <muda/ext/linear_system/device_dense_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>
#include <muda/ext/linear_system/device_doublet_vector.h>
#include <muda/ext/linear_system/device_bcoo_matrix.h>
#include <muda/ext/linear_system/device_bcoo_vector.h>
#include <muda/ext/linear_system/device_bsr_matrix.h>
#include <muda/ext/linear_system/device_csr_matrix.h>

#include <muda/cub/device/device_merge_sort.h>
#include <muda/cub/device/device_radix_sort.h>
#include <muda/cub/device/device_run_length_encode.h>
#include <muda/cub/device/device_scan.h>
#include <muda/cub/device/device_segmented_reduce.h>
#include <muda/cub/device/device_reduce.h>

#include <muda/type_traits/cuda_arch.h>
#include <muda/buffer/device_var.h>

#include <muda/launch.h>


namespace muda
{
namespace details
{
    struct IntPair
    {
        int x;
        int y;
    };

    // A internal muda pair type to avoid int2 equality redefinition conflict
    constexpr bool operator==(const IntPair& l, const IntPair& r)
    {
        return l.x == r.x && l.y == r.y;
    }
}  // namespace details
}  // namespace muda


namespace muda
{
namespace details
{
    class MatrixFormatConverterBase
    {
      protected:
        LinearSystemHandles& m_handles;
        cudaDataType_t       m_data_type;
        int                  m_M;
        int                  m_N;

      public:
        MatrixFormatConverterBase(LinearSystemHandles& context, cudaDataType_t data_type, int M, int N)
            : m_handles(context)
            , m_data_type(data_type)
            , m_M(M)
            , m_N(N)
        {
        }

        virtual ~MatrixFormatConverterBase() = default;

        auto dim_N() const { return m_N; }
        auto dim_M() const { return m_M; }
        auto data_type() const { return m_data_type; }
        auto cublas() const { return m_handles.cublas(); }
        auto cusparse() const { return m_handles.cusparse(); }
        auto cusolver_sp() const { return m_handles.cusolver_sp(); }
        auto cusolver_dn() const { return m_handles.cusolver_dn(); }

        template <typename T>
        void loose_resize(DeviceBuffer<T>& buf, size_t new_size)
        {
            if(buf.capacity() < new_size)
                buf.reserve(new_size * m_handles.reserve_ratio());
            buf.resize(new_size);
        }
    };

    template <typename T, int M, int N>
    class MatrixFormatConverter : public MatrixFormatConverterBase
    {
        using MatrixValueT = typename DeviceTripletMatrix<T, M, N>::ValueT;
        using VectorValueT = typename DeviceDoubletVector<T, M>::ValueT;

        MatrixValueT MatrixValueTZero() const
        {
            if constexpr(M > 1 || N > 1)
            {
                return MatrixValueT::Zero().eval();
            }
            else
            {
                return MatrixValueT{0};
            }
        }

        VectorValueT VectorValueTZero() const
        {
            if constexpr(M > 1)
            {
                return VectorValueT::Zero().eval();
            }
            else
            {
                return VectorValueT{0};
            }
        }

        DeviceBuffer<int> sort_index;
        DeviceBuffer<int> sort_index_input;
        DeviceBuffer<int> sort_index_tmp;

        DeviceBuffer<int> col_tmp;
        DeviceBuffer<int> row_tmp;

        DeviceBCOOMatrix<T, M, N> temp_bcoo_matrix;
        DeviceBCOOVector<T, N>    temp_bcoo_vector;

        DeviceBuffer<int> unique_indices;
        DeviceBuffer<int> unique_counts;
        DeviceBuffer<int> offsets;

        DeviceVar<int> count;

        DeviceBuffer<IntPair> ij_pairs;
        DeviceBuffer<int64_t> ij_hash;
        DeviceBuffer<int64_t> ij_hash_input;
        DeviceBuffer<IntPair> unique_ij_pairs;

        muda::DeviceBuffer<MatrixValueT> blocks_sorted;
        DeviceBuffer<MatrixValueT>       unique_blocks;
        DeviceBuffer<VectorValueT>       unique_segments;
        DeviceBuffer<VectorValueT>       temp_segments;

        DeviceBuffer<T> unique_values;

      public:
        MatrixFormatConverter(LinearSystemHandles& handles)
            : MatrixFormatConverterBase(handles, cuda_data_type<T>(), M, N)
        {
        }

        virtual ~MatrixFormatConverter() = default;


        // Triplet -> BCOO
        void convert(const DeviceTripletMatrix<T, M, N>& from,
                     DeviceBCOOMatrix<T, M, N>&          to)
        {
            to.reshape(from.rows(), from.cols());
            to.resize_triplets(from.triplet_count());


            if(to.triplet_count() == 0)
                return;

            if constexpr(M <= 3 && N <= 3)
            {
                radix_sort_indices_and_blocks(from, to);
                make_unique_indices_and_blocks(from, to);
            }
            else
            {
                merge_sort_indices_and_blocks(from, to);
                make_unique_indices(from, to);
                make_unique_blocks(from, to);
            }
        }

        void radix_sort_indices_and_blocks(const DeviceTripletMatrix<T, M, N>& from,
                                           DeviceBCOOMatrix<T, M, N>& to)
        {
            auto src_row_indices = from.row_indices();
            auto src_col_indices = from.col_indices();
            auto src_blocks      = from.values();

            loose_resize(ij_hash_input, src_row_indices.size());
            loose_resize(sort_index_input, src_row_indices.size());

            loose_resize(ij_hash, src_row_indices.size());
            loose_resize(sort_index, src_row_indices.size());
            ij_pairs.resize(src_row_indices.size());


            // hash ij
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(src_row_indices.size(),
                       [row_indices = src_row_indices.cviewer().name("row_indices"),
                        col_indices = src_col_indices.cviewer().name("col_indices"),
                        ij_hash = ij_hash_input.viewer().name("ij_hash"),
                        sort_index = sort_index_input.viewer().name("sort_index")] __device__(int i) mutable
                       {
                           ij_hash(i) = (int64_t{row_indices(i)} << 32)
                                        + int64_t{col_indices(i)};
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

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(dst_row_indices.size(),
                       [ij_hash = ij_hash.viewer().name("ij_hash"),
                        ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
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
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(src_blocks.size(),
                           [src_blocks = src_blocks.cviewer().name("blocks"),
                            sort_index = sort_index.cviewer().name("sort_index"),
                            dst_blocks = blocks_sorted.viewer().name(
                                "block_values")] __device__(int i) mutable
                           { dst_blocks(i) = src_blocks(sort_index(i)); });
            }
        }


        void make_unique_indices_and_blocks(const DeviceTripletMatrix<T, M, N>& from,
                                            DeviceBCOOMatrix<T, M, N>& to)
        {
            // alias to reuse the memory
            auto& unique_ij_hash = ij_hash_input;

            muda::DeviceReduce().ReduceByKey(
                ij_hash.data(),
                unique_ij_hash.data(),
                blocks_sorted.data(),
                to.values().data(),
                count.data(),
                [] CUB_RUNTIME_FUNCTION(const MatrixValueT& l, const MatrixValueT& r) -> MatrixValueT
                { return l + r; },
                ij_hash.size());

            int h_count = count;

            to.resize_triplets(h_count);

            // set ij_hash back to row_indices and col_indices
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(to.row_indices().size(),
                       [ij_hash = unique_ij_hash.viewer().name("ij_hash"),
                        row_indices = to.row_indices().viewer().name("row_indices"),
                        col_indices = to.col_indices().viewer().name(
                            "col_indices")] __device__(int i) mutable
                       {
                           auto hash      = ij_hash(i);
                           auto row_index = static_cast<int>(hash >> 32);
                           auto col_index = static_cast<int>(hash & 0xFFFFFFFF);
                           row_indices(i) = row_index;
                           col_indices(i) = col_index;
                       });
        }

        void merge_sort_indices_and_blocks(const DeviceTripletMatrix<T, M, N>& from,
                                           DeviceBCOOMatrix<T, M, N>& to)
        {
            using namespace muda;

            auto src_row_indices = from.row_indices();
            auto src_col_indices = from.col_indices();
            auto src_blocks      = from.values();

            loose_resize(sort_index, src_row_indices.size());
            loose_resize(ij_pairs, src_row_indices.size());

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(src_row_indices.size(),
                       [row_indices = src_row_indices.cviewer().name("row_indices"),
                        col_indices = src_col_indices.cviewer().name("col_indices"),
                        ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
                       {
                           ij_pairs(i).x = row_indices(i);
                           ij_pairs(i).y = col_indices(i);
                       });

            ParallelFor()
                .file_line(__FILE__, __LINE__)  //
                .apply(src_row_indices.size(),
                       [sort_index = sort_index.viewer().name("sort_index")] __device__(
                           int i) mutable { sort_index(i) = i; });

            DeviceMergeSort().SortPairs(
                ij_pairs.data(),
                sort_index.data(),
                ij_pairs.size(),
                [] __device__(const IntPair& a, const IntPair& b)
                { return a.x < b.x || (a.x == b.x && a.y < b.y); });


            // set ij_pairs back to row_indices and col_indices

            auto dst_row_indices = to.row_indices();
            auto dst_col_indices = to.col_indices();

            ParallelFor()
                .kernel_name("set col row indices")
                .apply(dst_row_indices.size(),
                       [row_indices = dst_row_indices.viewer().name("row_indices"),
                        col_indices = dst_col_indices.viewer().name("col_indices"),
                        ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
                       {
                           row_indices(i) = ij_pairs(i).x;
                           col_indices(i) = ij_pairs(i).y;
                       });


            // sort the block values

            loose_resize(unique_blocks, from.m_values.size());

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(src_blocks.size(),
                       [src_blocks = src_blocks.cviewer().name("blocks"),
                        sort_index = sort_index.cviewer().name("sort_index"),
                        dst_blocks = unique_blocks.viewer().name("block_values")] __device__(int i) mutable
                       { dst_blocks(i) = src_blocks(sort_index(i)); });
        }

        void make_unique_indices(const DeviceTripletMatrix<T, M, N>& from,
                                 DeviceBCOOMatrix<T, M, N>&          to)
        {
            using namespace muda;

            auto& row_indices = to.m_row_indices;
            auto& col_indices = to.m_col_indices;

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


            muda::ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(unique_counts.size(),
                       [unique_ij_pairs = unique_ij_pairs.viewer().name("unique_ij_pairs"),
                        row_indices = row_indices.viewer().name("row_indices"),
                        col_indices = col_indices.viewer().name("col_indices")] __device__(int i) mutable
                       {
                           row_indices(i) = unique_ij_pairs(i).x;
                           col_indices(i) = unique_ij_pairs(i).y;
                       });

            row_indices.resize(h_count);
            col_indices.resize(h_count);
        }

        void make_unique_blocks(const DeviceTripletMatrix<T, M, N>& from,
                                DeviceBCOOMatrix<T, M, N>&          to)
        {
            using namespace muda;

            auto& row_indices = to.m_row_indices;
            auto& values      = to.m_values;
            values.resize(row_indices.size());
            // first we add the offsets to counts, to get the offset_ends

            Launch()
                .file_line(__FILE__, __LINE__)
                .apply([offsets = offsets.viewer().name("offset"),
                        counts  = unique_counts.cviewer().name("counts"),
                        last    = unique_counts.size() - 1] __device__() mutable
                       { offsets(last + 1) = offsets(last) + counts(last); });

            // auto& begin_offset = offsets;
            // auto& end_offset = unique_counts;  // already contains the offset_ends

            // then we do a segmented reduce to get the unique blocks

            DeviceSegmentedReduce().Reduce(
                unique_blocks.data(),
                values.data(),
                values.size(),
                offsets.data(),
                offsets.data() + 1,
                [] __host__ __device__(const MatrixValueT& a, const MatrixValueT& b) -> MatrixValueT
                { return a + b; },
                MatrixValueTZero());
        }


        // BCOO -> Dense Matrix
        void convert(const DeviceBCOOMatrix<T, M, N>& from,
                     DeviceDenseMatrix<T>&            to,
                     bool                             clear_dense_matrix = true)
        {
            using namespace muda;
            auto dst_rows = M * from.rows();
            auto dst_cols = N * from.cols();
            to.reshape(dst_rows, dst_cols);

            if(clear_dense_matrix)
                to.fill(0);

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(from.values().size(),
                       [triplets = from.cviewer().name("src_sparse_matrix"),
                        dst = to.viewer().name("dst_dense_matrix")] __device__(int i) mutable
                       {
                           auto triplet = triplets(i);
                           auto row     = triplet.row_index * M;
                           auto col     = triplet.col_index * N;

                           if constexpr(M == 1 && N == 1)
                           {
                               dst(row, col) += triplet.value;
                           }
                           else
                           {
                               dst.block<M, N>(row, col).as_eigen() += triplet.value;
                           }
                       });
        }

        // BCOO -> COO
        void convert(const DeviceBCOOMatrix<T, M, N>& from, DeviceCOOMatrix<T>& to)
            MUDA_REQUIRES(M* N > 1)
        {
            static_assert(M * N > 1, "M * N must be greater than 1");
            expand_blocks(from, to);
            sort_indices_and_values(from, to);
        }

        void expand_blocks(const DeviceBCOOMatrix<T, M, N>& from,
                           DeviceCOOMatrix<T>& to) MUDA_REQUIRES(M* N > 1)
        {
            static_assert(M * N > 1, "M * N  must be greater than 1");

            constexpr int MN = M * N;

            to.reshape(from.rows() * M, from.cols() * N);
            to.resize_triplets(from.non_zeros() * MN);

            auto& dst_row_indices = to.m_row_indices;
            auto& dst_col_indices = to.m_col_indices;
            auto& dst_values      = to.m_values;

            auto& src_row_indices = from.m_row_indices;
            auto& src_col_indices = from.m_col_indices;
            auto& src_values      = from.m_values;


            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(src_row_indices.size(),
                       [src_row_indices = src_row_indices.cviewer().name("src_row_indices"),
                        src_col_indices = src_col_indices.cviewer().name("src_col_indices"),
                        src_values = src_values.cviewer().name("src_values"),

                        dst_row_indices = dst_row_indices.viewer().name("dst_row_indices"),
                        dst_col_indices = dst_col_indices.viewer().name("dst_col_indices"),
                        dst_values = dst_values.viewer().name("dst_values")] __device__(int i) mutable
                       {
                           auto src_row_index = src_row_indices(i);
                           auto src_col_index = src_col_indices(i);
                           auto src_value     = src_values(i);

                           auto row = src_row_index * M;
                           auto col = src_col_index * N;

                           auto index = i * MN;
#pragma unroll
                           for(int r = 0; r < M; ++r)
                           {
#pragma unroll
                               for(int c = 0; c < N; ++c)
                               {
                                   dst_row_indices(index) = row + r;
                                   dst_col_indices(index) = col + c;
                                   dst_values(index)      = src_value(r, c);
                                   ++index;
                               }
                           }
                       });
        }

        void sort_indices_and_values(const DeviceBCOOMatrix<T, M, N>& from,
                                     DeviceCOOMatrix<T>& to) MUDA_REQUIRES(M* N > 1)
        {
            static_assert(M * N > 1, "M * N must be greater than 1");

            auto& row_indices = to.m_row_indices;
            auto& col_indices = to.m_col_indices;
            auto& values      = to.m_values;

            ij_pairs.resize(row_indices.size());

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(row_indices.size(),
                       [row_indices = row_indices.cviewer().name("row_indices"),
                        col_indices = col_indices.cviewer().name("col_indices"),
                        ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
                       {
                           ij_pairs(i).x = row_indices(i);
                           ij_pairs(i).y = col_indices(i);
                       });

            DeviceMergeSort().SortPairs(
                ij_pairs.data(),
                to.m_values.data(),
                ij_pairs.size(),
                [] __device__(const IntPair& a, const IntPair& b)
                { return a.x < b.x || (a.x == b.x && a.y < b.y); });

            // set ij_pairs back to row_indices and col_indices

            auto dst_row_indices = to.row_indices();
            auto dst_col_indices = to.col_indices();

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(dst_row_indices.size(),
                       [row_indices = dst_row_indices.viewer().name("row_indices"),
                        col_indices = dst_col_indices.viewer().name("col_indices"),
                        ij_pairs = ij_pairs.viewer().name("ij_pairs")] __device__(int i) mutable
                       {
                           row_indices(i) = ij_pairs(i).x;
                           col_indices(i) = ij_pairs(i).y;
                       });
        }

        // BCOO -> BSR
        void convert(const DeviceBCOOMatrix<T, M, N>& from, DeviceBSRMatrix<T, N>& to)
            MUDA_REQUIRES(M == N)
        {
            static_assert(M == N, "M must be equal to N");

            calculate_block_offsets(from, to);

            to.m_col_indices = from.m_col_indices;
            to.m_values      = from.m_values;
        }

        void convert(DeviceBCOOMatrix<T, M, N>&& from, DeviceBSRMatrix<T, N>& to)
            MUDA_REQUIRES(M == N)
        {
            static_assert(M == N, "M must be equal to N");

            calculate_block_offsets(from, to);

            to.m_col_indices = std::move(from.m_col_indices);
            to.m_values      = std::move(from.m_values);
        }

        void calculate_block_offsets(const DeviceBCOOMatrix<T, M, N>& from,
                                     DeviceBSRMatrix<T, N>& to) MUDA_REQUIRES(M == N)
        {
            static_assert(M == N, "M must be equal to N");

            to.reshape(from.rows(), from.cols());

            auto& dst_row_offsets = to.m_row_offsets;

            // alias the offsets to the col_counts_per_row(reuse)
            auto& col_counts_per_row = offsets;
            col_counts_per_row.resize(to.m_row_offsets.size());
            col_counts_per_row.fill(0);

            unique_indices.resize(from.non_zeros());
            unique_counts.resize(from.non_zeros());

            // run length encode the row
            DeviceRunLengthEncode().Encode(from.m_row_indices.data(),
                                           unique_indices.data(),
                                           unique_counts.data(),
                                           count.data(),
                                           from.non_zeros());
            int h_count = count;

            unique_indices.resize(h_count);
            unique_counts.resize(h_count);

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(unique_counts.size(),
                       [unique_indices = unique_indices.cviewer().name("offset"),
                        counts = unique_counts.viewer().name("counts"),
                        col_counts_per_row = col_counts_per_row.viewer().name(
                            "col_counts_per_row")] __device__(int i) mutable
                       {
                           auto row                = unique_indices(i);
                           col_counts_per_row(row) = counts(i);
                       });

            // calculate the offsets
            DeviceScan().ExclusiveSum(col_counts_per_row.data(),
                                      dst_row_offsets.data(),
                                      col_counts_per_row.size());
        }

        // Doublet -> BCOO
        void convert(const DeviceDoubletVector<T, N>& from, DeviceBCOOVector<T, N>& to)
        {
            to.reshape(from.count());
            to.resize_doublets(from.doublet_count());
            merge_sort_indices_and_segments(from, to);
            make_unique_indices(from, to);
            make_unique_segments(from, to);
        }

        void merge_sort_indices_and_segments(const DeviceDoubletVector<T, N>& from,
                                             DeviceBCOOVector<T, N>& to)
        {
            using namespace muda;

            auto& indices = sort_index;  // alias sort_index to index

            // copy as temp
            indices       = from.m_indices;
            temp_segments = from.m_values;

            DeviceMergeSort().SortPairs(indices.data(),
                                        temp_segments.data(),
                                        indices.size(),
                                        [] __device__(const int& a, const int& b)
                                        { return a < b; });
        }

        void make_unique_indices(const DeviceDoubletVector<T, N>& from,
                                 DeviceBCOOVector<T, N>&          to)
        {
            using namespace muda;

            auto& indices        = sort_index;  // alias sort_index to index
            auto& unique_indices = to.m_indices;

            loose_resize(unique_indices, indices.size());
            loose_resize(unique_counts, indices.size());

            DeviceRunLengthEncode().Encode(indices.data(),
                                           unique_indices.data(),
                                           unique_counts.data(),
                                           count.data(),
                                           indices.size());

            int h_count = count;

            unique_indices.resize(h_count);
            unique_counts.resize(h_count);

            loose_resize(offsets, unique_counts.size() + 1);

            DeviceScan().ExclusiveSum(
                unique_counts.data(), offsets.data(), unique_counts.size());

            // calculate the offset_ends, and set to the unique_counts

            // auto& begin_offset = offsets;

            Launch()
                .file_line(__FILE__, __LINE__)
                .apply([offset = offsets.viewer().name("offset"),
                        count  = unique_counts.cviewer().name("counts"),
                        last   = unique_counts.size() - 1] __device__() mutable
                       { offset(last + 1) = offset(last) + count(last); });
        }

        void make_unique_segments(const DeviceDoubletVector<T, N>& from,
                                  DeviceBCOOVector<T, N>&          to)
        {
            using namespace muda;

            auto& begin_offset = offsets;
            // auto& end_offset   = unique_counts;

            auto& unique_indices  = to.m_indices;
            auto& unique_segments = to.m_values;

            unique_segments.resize(unique_indices.size());

            DeviceSegmentedReduce().Reduce(
                temp_segments.data(),
                unique_segments.data(),
                unique_segments.size(),
                begin_offset.data(),
                begin_offset.data() + 1,
                [] __host__ __device__(const VectorValueT& a, const VectorValueT& b) -> VectorValueT
                { return a + b; },
                VectorValueTZero());
        }


        // BCOO -> Dense Vector
        void convert(const DeviceBCOOVector<T, N>& from,
                     DeviceDenseVector<T>&         to,
                     bool                          clear_dense_vector = true)
        {
            to.resize(N * from.count());
            set_unique_values_to_dense_vector(from, to, clear_dense_vector);
        }

        void set_unique_values_to_dense_vector(const DeviceBCOOVector<T, N>& from,
                                               DeviceDenseVector<T>& to,
                                               bool clear_dense_vector)
        {
            using namespace muda;

            if(clear_dense_vector)
                to.fill(0);

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(from.non_zeros(),
                       [unique_values = from.m_values.cviewer().name("unique_segments"),
                        unique_indices = from.m_indices.cviewer().name("unique_indices"),
                        dst = to.viewer().name("dst_dense_vector")] __device__(int i) mutable
                       {
                           auto index = unique_indices(i);

                           if constexpr(N == 1)
                           {
                               dst(index) += unique_values(i);
                           }
                           else
                           {
                               dst.segment<N>(index * N).as_eigen() += unique_values(i);
                           }
                       });
        }


        // Doublet -> Dense Vector
        void convert(const DeviceDoubletVector<T, N>& from,
                     DeviceDenseVector<T>&            to,
                     bool                             clear_dense_vector = true)
        {
            using namespace muda;

            to.resize(N * from.count());

            if(clear_dense_vector)
                to.fill(0);

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(from.doublet_count(),
                       [src = from.viewer().name("src_sparse_vector"),
                        dst = to.viewer().name("dst_dense_vector")] __device__(int i) mutable
                       {
                           auto&& [index, value] = src(i);
                           dst.segment<N>(index * N).atomic_add(value);
                       });
        }

        void bsr2csr(cusparseHandle_t         handle,
                     int                      mb,
                     int                      nb,
                     int                      blockDim,
                     cusparseMatDescr_t       descrA,
                     const T*                 bsrValA,
                     const int*               bsrRowPtrA,
                     const int*               bsrColIndA,
                     int                      nnzb,
                     DeviceCSRMatrix<T>&      to,
                     muda::DeviceBuffer<int>& row_offsets,
                     muda::DeviceBuffer<int>& col_indices,
                     muda::DeviceBuffer<T>&   values)
        {
            using namespace muda;
            cusparseDirection_t dir = CUSPARSE_DIRECTION_COLUMN;
            int                 m   = mb * blockDim;
            int nnz = nnzb * blockDim * blockDim;  // number of elements
            to.reshape(m, m);
            col_indices.resize(nnz);
            values.resize(nnz);

            if constexpr(std::is_same_v<T, float>)
            {
                checkCudaErrors(cusparseSbsr2csr(handle,
                                                 dir,
                                                 mb,
                                                 nb,
                                                 descrA,
                                                 bsrValA,
                                                 bsrRowPtrA,
                                                 bsrColIndA,
                                                 blockDim,
                                                 to.legacy_descr(),
                                                 values.data(),
                                                 row_offsets.data(),
                                                 col_indices.data()));
            }

            else if constexpr(std::is_same_v<T, double>)
            {
                checkCudaErrors(cusparseDbsr2csr(handle,
                                                 dir,
                                                 mb,
                                                 nb,
                                                 descrA,
                                                 bsrValA,
                                                 bsrRowPtrA,
                                                 bsrColIndA,
                                                 blockDim,
                                                 to.legacy_descr(),
                                                 values.data(),
                                                 row_offsets.data(),
                                                 col_indices.data()));
            }
        }

        // BSR -> CSR
        void convert(const DeviceBSRMatrix<T, N>& from, DeviceCSRMatrix<T>& to)
            MUDA_REQUIRES(N > 1)
        {
            static_assert(N > 1, "N must be greater than 1");
            using namespace muda;

            bsr2csr(cusparse(),
                    from.rows(),
                    from.cols(),
                    N,
                    from.legacy_descr(),
                    (const T*)from.m_values.data(),
                    from.m_row_offsets.data(),
                    from.m_col_indices.data(),
                    from.non_zeros(),
                    to,
                    to.m_row_offsets,
                    to.m_col_indices,
                    to.m_values);
        }
    };
}  // namespace details
}  // namespace muda
