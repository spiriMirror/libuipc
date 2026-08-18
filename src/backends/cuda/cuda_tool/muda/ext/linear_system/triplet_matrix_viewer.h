#pragma once
#include <string>
#include <muda/viewer/viewer_base.h>
#include <muda/buffer/device_buffer.h>
#include <muda/tools/cuda_vec_utils.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>


/*
* - 2024/2/23 remove viewer's subview, view's subview is enough
*/

namespace muda
{
template <bool IsConst, typename T, int M, int N = M>
class TripletMatrixViewerT : public ViewerBase<IsConst>
{
    using Base = ViewerBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U, int M_, int N_>
    friend class TripletMatrixViewerT;

    MUDA_VIEWER_COMMON_NAME(TripletMatrixViewerT);

  public:
    static constexpr bool IsBlockMatrix = (M > 1 || N > 1);
    using ValueT = std::conditional_t<IsBlockMatrix, Eigen::Matrix<T, M, N>, T>;

    using ConstViewer    = TripletMatrixViewerT<true, T, M, N>;
    using NonConstViewer = TripletMatrixViewerT<false, T, M, N>;
    using ThisViewer     = TripletMatrixViewerT<IsConst, T, M, N>;

    struct CTriplet
    {
        MUDA_GENERIC CTriplet(int row_index, int col_index, const ValueT& block)
            : row_index(row_index)
            , col_index(col_index)
            , value(block)
        {
        }

        int           row_index;
        int           col_index;
        const ValueT& value;
    };

    class Proxy
    {
        friend class TripletMatrixViewerT;
        const TripletMatrixViewerT& m_viewer;
        int                         m_index = 0;

      private:
        MUDA_GENERIC Proxy(const TripletMatrixViewerT& viewer, int index)
            : m_viewer(viewer)
            , m_index(index)
        {
        }

      public:
        MUDA_GENERIC auto read() && { return m_viewer.at(m_index); }

        MUDA_GENERIC
        void write(int row_index, int col_index, const ValueT& block) &&
        {
            auto index = m_viewer.get_index(m_index);

            m_viewer.check_in_submatrix(row_index, col_index);

            auto global_i = m_viewer.m_submatrix_offset.x + row_index;
            auto global_j = m_viewer.m_submatrix_offset.y + col_index;

            m_viewer.m_row_indices[index] = global_i;
            m_viewer.m_col_indices[index] = global_j;
            m_viewer.m_values[index]      = block;
        }

        MUDA_GENERIC ~Proxy() = default;
    };

  protected:
    // matrix info
    int m_total_rows = 0;
    int m_total_cols = 0;

    // triplet info
    int m_triplet_index_offset = 0;
    int m_triplet_count        = 0;
    int m_total_triplet_count  = 0;

    // sub matrix info
    int2 m_submatrix_offset = {0, 0};
    int2 m_submatrix_extent = {0, 0};

    // data
    auto_const_t<int>*    m_row_indices;
    auto_const_t<int>*    m_col_indices;
    auto_const_t<ValueT>* m_values;


  public:
    MUDA_GENERIC TripletMatrixViewerT() = default;

    MUDA_GENERIC TripletMatrixViewerT(int total_block_rows,
                                      int total_block_cols,
                                      int triplet_index_offset,
                                      int triplet_count,
                                      int total_triplet_count,

                                      int2 submatrix_offset,
                                      int2 submatrix_extent,

                                      auto_const_t<int>*    block_row_indices,
                                      auto_const_t<int>*    block_col_indices,
                                      auto_const_t<ValueT>* block_values)
        : m_total_rows(total_block_rows)
        , m_total_cols(total_block_cols)
        , m_triplet_index_offset(triplet_index_offset)
        , m_triplet_count(triplet_count)
        , m_total_triplet_count(total_triplet_count)
        , m_submatrix_offset(submatrix_offset)
        , m_submatrix_extent(submatrix_extent)
        , m_row_indices(block_row_indices)
        , m_col_indices(block_col_indices)
        , m_values(block_values)
    {
        MUDA_ASSERT(triplet_index_offset + triplet_count <= total_triplet_count,
                    "TripletMatrixViewer [%s:%s]: out of range, m_total_triplet_count=%d, "
                    "your triplet_index_offset=%d, triplet_count=%d. %s(%d)",
                    this->name(),
                    this->kernel_name(),
                    total_triplet_count,
                    triplet_index_offset,
                    triplet_count,
                    this->kernel_file(),
                    this->kernel_line());

        MUDA_ASSERT(submatrix_offset.x >= 0 && submatrix_offset.y >= 0,
                    "TripletMatrixViewer[%s:%s]: submatrix_offset is out of range, submatrix_offset.x=%d, submatrix_offset.y=%d. %s(%d)",
                    this->name(),
                    this->kernel_name(),
                    submatrix_offset.x,
                    submatrix_offset.y,
                    this->kernel_file(),
                    this->kernel_line());

        MUDA_ASSERT(submatrix_offset.x + submatrix_extent.x <= total_block_rows,
                    "TripletMatrixViewer[%s:%s]: submatrix is out of range, submatrix_offset.x=%d, submatrix_extent.x=%d, total_block_rows=%d. %s(%d)",
                    this->name(),
                    this->kernel_name(),
                    submatrix_offset.x,
                    submatrix_extent.x,
                    total_block_rows,
                    this->kernel_file(),
                    this->kernel_line());

        MUDA_ASSERT(submatrix_offset.y + submatrix_extent.y <= total_block_cols,
                    "TripletMatrixViewer[%s:%s]: submatrix is out of range, submatrix_offset.y=%d, submatrix_extent.y=%d, total_block_cols=%d. %s(%d)",
                    this->name(),
                    this->kernel_name(),
                    submatrix_offset.y,
                    submatrix_extent.y,
                    total_block_cols,
                    this->kernel_file(),
                    this->kernel_line());
    }

    template <bool OtherIsConst>
    MUDA_GENERIC TripletMatrixViewerT(const TripletMatrixViewerT<OtherIsConst, T, N>& other)
        : m_total_rows(other.m_total_rows)
        , m_total_cols(other.m_total_cols)
        , m_triplet_index_offset(other.m_triplet_index_offset)
        , m_triplet_count(other.m_triplet_count)
        , m_total_triplet_count(other.m_total_triplet_count)
        , m_submatrix_offset(other.m_submatrix_offset)
        , m_submatrix_extent(other.m_submatrix_extent)
        , m_row_indices(other.m_row_indices)
        , m_col_indices(other.m_col_indices)
        , m_values(other.m_values)
    {
    }

    MUDA_GENERIC ConstViewer as_const() const
    {
        return ConstViewer{m_total_rows,
                           m_total_cols,
                           m_triplet_index_offset,
                           m_triplet_count,
                           m_total_triplet_count,
                           m_submatrix_offset,
                           m_submatrix_extent,
                           m_row_indices,
                           m_col_indices,
                           m_values};
    }

    MUDA_GENERIC auto total_rows() const { return m_total_rows; }

    MUDA_GENERIC auto total_cols() const { return m_total_cols; }

    MUDA_GENERIC auto total_extent() const
    {
        return int2{m_total_rows, m_total_cols};
    }

    MUDA_GENERIC auto submatrix_offset() const { return m_submatrix_offset; }

    MUDA_GENERIC auto extent() const { return m_submatrix_extent; }

    MUDA_GENERIC auto triplet_count() const { return m_triplet_count; }

    MUDA_GENERIC auto tripet_index_offset() const
    {
        return m_triplet_index_offset;
    }
    MUDA_GENERIC auto total_triplet_count() const
    {
        return m_total_triplet_count;
    }

    MUDA_GENERIC auto operator()(int i) const
    {
        if constexpr(IsConst)
        {
            return at(i);
        }
        else
        {
            return Proxy{*this, i};
        }
    }

  protected:
    MUDA_GENERIC MUDA_INLINE CTriplet at(int i) const noexcept
    {
        auto index    = get_index(i);
        auto global_i = m_row_indices[index];
        auto global_j = m_col_indices[index];
        auto sub_i    = global_i - m_submatrix_offset.x;
        auto sub_j    = global_j - m_submatrix_offset.y;
        check_in_submatrix(sub_i, sub_j);
        return CTriplet{sub_i, sub_j, m_values[index]};
    }

    MUDA_INLINE MUDA_GENERIC int get_index(int i) const noexcept
    {

        MUDA_KERNEL_ASSERT(i >= 0 && i < m_triplet_count,
                           "TripletMatrixViewer [%s:%s]: triplet_index out of range, block_count=%d, your index=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_triplet_count,
                           i,
                           this->kernel_file(),
                           this->kernel_line());
        auto index = i + m_triplet_index_offset;
        return index;
    }

    MUDA_INLINE MUDA_GENERIC void check_in_submatrix(int i, int j) const noexcept
    {
        MUDA_KERNEL_ASSERT(i >= 0 && i < m_submatrix_extent.x,
                           "TripletMatrixViewer [%s:%s]: row index out of submatrix range,  submatrix_extent.x=%d, your i=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_submatrix_extent.x,
                           i,
                           this->kernel_file(),
                           this->kernel_line());

        MUDA_KERNEL_ASSERT(j >= 0 && j < m_submatrix_extent.y,
                           "TripletMatrixViewer [%s:%s]: col index out of submatrix range,  submatrix_extent.y=%d, your j=%d. %s(%d)",
                           this->name(),
                           this->kernel_name(),
                           m_submatrix_extent.y,
                           j,
                           this->kernel_file(),
                           this->kernel_line());
    }
};

template <typename T, int M, int N = M>
using TripletMatrixViewer = TripletMatrixViewerT<false, T, M, N>;

template <typename T, int M, int N = M>
using CTripletMatrixViewer = TripletMatrixViewerT<true, T, M, N>;
}  // namespace muda

#include "details/triplet_matrix_viewer.inl"
