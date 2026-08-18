#pragma once
#include <muda/ext/linear_system/triplet_matrix_view.h>
#include <muda/ext/linear_system/bcoo_matrix_viewer.h>

namespace muda
{
template <typename T, int M, int N = M>
using BCOOMatrixView = TripletMatrixView<T, M, N>;
template <typename T, int M, int N = M>
using CBCOOMatrixView = CTripletMatrixView<T, M, N>;
}  // namespace muda

namespace muda
{
template <bool IsConst, typename Ty>
class COOMatrixViewT : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U>
    friend class COOMatrixViewT;

  public:
    static_assert(!std::is_const_v<Ty>, "Ty must be non-const");
    using NonConstView = COOMatrixViewT<false, Ty>;
    using ConstView    = COOMatrixViewT<true, Ty>;
    using ThisView     = COOMatrixViewT<IsConst, Ty>;

  protected:
    // matrix info
    int m_rows = 0;
    int m_cols = 0;

    // triplet info
    int m_triplet_index_offset = 0;
    int m_triplet_count        = 0;
    int m_total_triplet_count  = 0;

    // sub matrix info
    int2 m_submatrix_offset = {0, 0};
    int2 m_submatrix_extent = {0, 0};

    // data
    auto_const_t<int>* m_row_indices;
    auto_const_t<int>* m_col_indices;
    auto_const_t<Ty>*  m_values;

    mutable cusparseMatDescr_t   m_legacy_descr = nullptr;
    mutable cusparseSpMatDescr_t m_descr        = nullptr;
    bool                         m_trans        = false;

  public:
    MUDA_GENERIC COOMatrixViewT() = default;

    MUDA_GENERIC COOMatrixViewT(int                  rows,
                                int                  cols,
                                int                  triplet_index_offset,
                                int                  triplet_count,
                                int                  total_triplet_count,
                                int2                 submatrix_offset,
                                int2                 submatrix_extent,
                                auto_const_t<int>*   row_indices,
                                auto_const_t<int>*   col_indices,
                                auto_const_t<Ty>*    values,
                                cusparseSpMatDescr_t descr,
                                cusparseMatDescr_t   legacy_descr,
                                bool                 trans)

        : m_rows(rows)
        , m_cols(cols)
        , m_triplet_index_offset(triplet_index_offset)
        , m_triplet_count(triplet_count)
        , m_total_triplet_count(total_triplet_count)
        , m_row_indices(row_indices)
        , m_col_indices(col_indices)
        , m_values(values)
        , m_submatrix_offset(submatrix_offset)
        , m_submatrix_extent(submatrix_extent)
        , m_descr(descr)
        , m_legacy_descr(legacy_descr)
        , m_trans(trans)
    {
        MUDA_KERNEL_ASSERT(triplet_index_offset + triplet_count <= total_triplet_count,
                           "COOMatrixView: out of range, m_total_triplet_count=%d, "
                           "your triplet_index_offset=%d, triplet_count=%d",
                           total_triplet_count,
                           triplet_index_offset,
                           triplet_count);


        MUDA_KERNEL_ASSERT(submatrix_offset.x >= 0 && submatrix_offset.y >= 0,
                           "TripletMatrixView: submatrix_offset is out of range, submatrix_offset.x=%d, submatrix_offset.y=%d",
                           submatrix_offset.x,
                           submatrix_offset.y);

        MUDA_KERNEL_ASSERT(submatrix_offset.x + submatrix_extent.x <= rows,
                           "TripletMatrixView: submatrix is out of range, submatrix_offset.x=%d, submatrix_extent.x=%d, total_block_rows=%d",
                           submatrix_offset.x,
                           submatrix_extent.x,
                           rows);

        MUDA_KERNEL_ASSERT(submatrix_offset.y + submatrix_extent.y <= cols,
                           "TripletMatrixView: submatrix is out of range, submatrix_offset.y=%d, submatrix_extent.y=%d, total_block_cols=%d",
                           submatrix_offset.y,
                           submatrix_extent.y,
                           cols);
    }

    MUDA_GENERIC COOMatrixViewT(int                  rows,
                                int                  cols,
                                int                  total_triplet_count,
                                auto_const_t<int>*   row_indices,
                                auto_const_t<int>*   col_indices,
                                auto_const_t<Ty>*    values,
                                cusparseSpMatDescr_t descr,
                                cusparseMatDescr_t   legacy_descr,
                                bool                 trans)
        : COOMatrixViewT(rows,
                         cols,
                         0,
                         total_triplet_count,
                         total_triplet_count,
                         {0, 0},
                         {rows, cols},
                         row_indices,
                         col_indices,
                         values,
                         descr,
                         legacy_descr,
                         trans)
    {
    }

    template <bool OtherIsConst>
    MUDA_GENERIC COOMatrixViewT(const COOMatrixViewT<OtherIsConst, Ty>& other)
        : m_rows(other.m_rows)
        , m_cols(other.m_cols)
        , m_triplet_index_offset(other.m_triplet_index_offset)
        , m_triplet_count(other.m_triplet_count)
        , m_total_triplet_count(other.m_total_triplet_count)
        , m_submatrix_offset(other.m_submatrix_offset)
        , m_submatrix_extent(other.m_submatrix_extent)
        , m_row_indices(other.m_row_indices)
        , m_col_indices(other.m_col_indices)
        , m_values(other.m_values)
        , m_descr(other.m_descr)
        , m_legacy_descr(other.m_legacy)
    {
    }

    MUDA_GENERIC auto as_const() const
    {
        return ConstView{m_rows,
                         m_cols,
                         m_triplet_index_offset,
                         m_triplet_count,
                         m_total_triplet_count,
                         m_submatrix_offset,
                         m_submatrix_extent,
                         m_row_indices,
                         m_col_indices,
                         m_values,
                         m_descr,
                         m_legacy_descr,
                         m_trans};
    }

    MUDA_GENERIC auto cviewer() const
    {
        MUDA_KERNEL_ASSERT(!m_trans,
                           "COOMatrixView: cviewer() is not supported for "
                           "transposed matrix, please use a non-transposed view of this matrix");
        return CTripletMatrixViewer<Ty, 1, 1>{m_rows,
                                              m_cols,
                                              m_triplet_index_offset,
                                              m_triplet_count,
                                              m_total_triplet_count,
                                              m_submatrix_offset,
                                              m_submatrix_extent,
                                              m_row_indices,
                                              m_col_indices,
                                              m_values};
    }

    MUDA_GENERIC auto viewer()
    {
        MUDA_ASSERT(!m_trans,
                    "COOMatrixView: viewer() is not supported for "
                    "transposed matrix, please use a non-transposed view of this matrix");
        return TripletMatrixViewer<Ty, 1, 1>{m_rows,
                                             m_cols,
                                             m_triplet_index_offset,
                                             m_triplet_count,
                                             m_total_triplet_count,
                                             m_submatrix_offset,
                                             m_submatrix_extent,
                                             m_row_indices,
                                             m_col_indices,
                                             m_values};
    }

    // const access
    auto values() const { return m_values; }
    auto row_indices() const { return m_row_indices; }
    auto col_indices() const { return m_col_indices; }

    auto rows() const { return m_rows; }
    auto cols() const { return m_cols; }
    auto triplet_count() const { return m_triplet_count; }
    auto tripet_index_offset() const { return m_triplet_index_offset; }
    auto total_triplet_count() const { return m_total_triplet_count; }
    auto is_trans() const { return m_trans; }

    auto legacy_descr() const { return m_legacy_descr; }
    auto descr() const { return m_descr; }
};

template <typename Ty>
using COOMatrixView = COOMatrixViewT<false, Ty>;
template <typename Ty>
using CCOOMatrixView = COOMatrixViewT<true, Ty>;
}  // namespace muda

namespace muda
{
template <typename T>
struct read_only_view<COOMatrixView<T>>
{
    using type = CCOOMatrixView<T>;
};

template <typename T>
struct read_write_view<CCOOMatrixView<T>>
{
    using type = COOMatrixView<T>;
};
}  // namespace muda
#include "details/bcoo_matrix_view.inl"
