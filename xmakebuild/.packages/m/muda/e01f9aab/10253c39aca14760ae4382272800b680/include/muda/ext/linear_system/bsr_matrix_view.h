#pragma once
#include <cusparse_v2.h>
#include <muda/view/view_base.h>

namespace muda
{
template <bool IsConst, typename Ty, int N>
class BSRMatrixViewT : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U, int M>
    friend class BSRMatrixViewT;

  public:
    static_assert(!std::is_const_v<Ty>, "Ty must be non-const");

    using ValueT    = std::conditional_t<N == 1, Ty, Eigen::Matrix<Ty, N, N>>;
    using ConstView = BSRMatrixViewT<true, Ty, N>;
    using NonConstView = BSRMatrixViewT<false, Ty, N>;
    using ThisView     = BSRMatrixViewT<IsConst, Ty, N>;

  protected:
    // data
    int m_row = 0;
    int m_col = 0;

    auto_const_t<int>*    m_row_offsets = nullptr;
    auto_const_t<int>*    m_col_indices = nullptr;
    auto_const_t<ValueT>* m_values      = nullptr;
    int                   m_non_zeros   = 0;

    mutable cusparseMatDescr_t   m_legacy_descr = nullptr;
    mutable cusparseSpMatDescr_t m_descr        = nullptr;

    bool m_trans = false;

  public:
    MUDA_GENERIC BSRMatrixViewT() noexcept = default;
    MUDA_GENERIC BSRMatrixViewT(int                   row,
                                int                   col,
                                auto_const_t<int>*    block_row_offsets,
                                auto_const_t<int>*    block_col_indices,
                                auto_const_t<ValueT>* block_values,
                                int                   non_zeros,
                                cusparseSpMatDescr_t  descr,
                                cusparseMatDescr_t    legacy_descr,
                                bool                  trans) noexcept
        : m_row(row)
        , m_col(col)
        , m_row_offsets(block_row_offsets)
        , m_col_indices(block_col_indices)
        , m_values(block_values)
        , m_non_zeros(non_zeros)
        , m_descr(descr)
        , m_legacy_descr(legacy_descr)
        , m_trans(trans)

    {
    }

    template <bool OtherIsConst>
    MUDA_GENERIC BSRMatrixViewT(const BSRMatrixViewT<OtherIsConst, Ty, N>& other) noexcept
        MUDA_REQUIRES(IsConst)
        : m_row(other.m_row)
        , m_col(other.m_col)
        , m_row_offsets(other.m_row_offsets)
        , m_col_indices(other.m_col_indices)
        , m_values(other.m_values)
        , m_non_zeros(other.m_non_zeros)
        , m_descr(other.m_descr)
        , m_legacy_descr(other.m_legacy)
    {
        static_assert(IsConst);
    }

    MUDA_GENERIC ConstView as_const() const
    {
        return ConstView{
            m_row, m_col, m_row_offsets, m_col_indices, m_values, m_non_zeros, m_descr, m_legacy_descr, m_trans};
    }

    MUDA_GENERIC auto values() const { return m_values; }
    MUDA_GENERIC auto row_offsets() const { return m_row_offsets; }
    MUDA_GENERIC auto col_indices() const { return m_col_indices; }

    MUDA_GENERIC auto rows() const { return m_row; }
    MUDA_GENERIC auto cols() const { return m_col; }
    MUDA_GENERIC auto non_zeros() const { return m_non_zeros; }

    MUDA_GENERIC auto legacy_descr() const { return m_legacy_descr; }
    MUDA_GENERIC auto descr() const { return m_descr; }
    MUDA_GENERIC auto is_trans() const { return m_trans; }

    MUDA_GENERIC auto T() const
    {
        return ThisView{
            m_row, m_col, m_row_offsets, m_col_indices, m_values, m_non_zeros, m_descr, m_legacy_descr, !m_trans};
    }
};

template <typename Ty, int N>
using BSRMatrixView = BSRMatrixViewT<false, Ty, N>;
template <typename Ty, int N>
using CBSRMatrixView = BSRMatrixViewT<true, Ty, N>;
}  // namespace muda

namespace muda
{
template <typename Ty, int N>
struct read_only_view<BSRMatrixView<Ty, N>>
{
    using type = CBSRMatrixView<Ty, N>;
};

template <typename Ty, int N>
struct read_write_view<CBSRMatrixView<Ty, N>>
{
    using type = BSRMatrixView<Ty, N>;
};
}  // namespace muda


#include "details/bsr_matrix_view.inl"
