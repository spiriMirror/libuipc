#pragma once
#include <muda/ext/linear_system/dense_matrix_viewer.h>
#include <muda/buffer/buffer_2d_view.h>
#include <muda/view/view_base.h>
namespace muda
{
template <bool IsConst, typename Ty>
class DenseMatrixViewT : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U>
    friend class DenseMatrixViewT;

  public:
    static_assert(std::is_same_v<Ty, float> || std::is_same_v<Ty, double>,
                  "now only support real number");

    using ConstView    = DenseMatrixViewT<true, Ty>;
    using NonConstView = DenseMatrixViewT<false, Ty>;
    using ThisView     = DenseMatrixViewT<IsConst, Ty>;

    using CBuffer2DView = CBuffer2DView<Ty>;
    using Buffer2DView  = Buffer2DView<Ty>;
    using ThisBuffer2DView = std::conditional_t<IsConst, CBuffer2DView, Buffer2DView>;

    using CViewer    = CDenseMatrixViewer<Ty>;
    using Viewer     = DenseMatrixViewer<Ty>;
    using ThisViewer = std::conditional_t<IsConst, CViewer, Viewer>;

  protected:
    ThisBuffer2DView m_view;
    size_t           m_row   = 0;
    size_t           m_col   = 0;
    bool             m_trans = false;
    bool             m_sym   = false;

  public:
    MUDA_GENERIC DenseMatrixViewT(ThisBuffer2DView view,
                                  size_t           row,
                                  size_t           col,
                                  bool             trans = false,
                                  bool sym = false) MUDA_NOEXCEPT : m_view(view),
                                                                    m_row(row),
                                                                    m_col(col),
                                                                    m_trans(trans),
                                                                    m_sym(sym)
    {
    }

    template <bool OtherIsConst>
    MUDA_GENERIC DenseMatrixViewT(const DenseMatrixViewT<OtherIsConst, Ty>& other) MUDA_NOEXCEPT
        MUDA_REQUIRES(IsConst)
        : m_view(other.m_view)
        , m_row(other.m_row)
        , m_col(other.m_col)
    {
        static_assert(IsConst);
    }

    MUDA_GENERIC auto as_const() const MUDA_NOEXCEPT
    {
        return ConstView{m_view, m_row, m_col, m_trans, m_sym};
    }

    MUDA_GENERIC auto T() MUDA_NOEXCEPT
    {
        return ThisView{m_view, m_row, m_col, !m_trans, m_sym};
    }

    MUDA_GENERIC auto viewer() MUDA_NOEXCEPT
    {
        MUDA_ASSERT(!m_trans,
                    "DenseMatrixViewer doesn't support transpose, "
                    "please use the original matrix to create a viewer");
        return ThisViewer{m_view, 0, 0, m_row, m_col};
    }

    MUDA_GENERIC auto cviewer() MUDA_NOEXCEPT
    {
        MUDA_ASSERT(!m_trans,
                    "DenseMatrixViewer doesn't support transpose, "
                    "please use the original matrix to create a viewer");
        return CViewer{m_view, 0, 0, m_row, m_col};
    }


    MUDA_GENERIC auto data() const MUDA_NOEXCEPT
    {
        return m_view.origin_data();
    }
    MUDA_GENERIC auto buffer_view() const MUDA_NOEXCEPT { return m_view; }

    MUDA_GENERIC bool is_trans() const MUDA_NOEXCEPT { return m_trans; }

    MUDA_GENERIC bool is_sym() const MUDA_NOEXCEPT { return m_sym; }

    MUDA_GENERIC size_t row() const MUDA_NOEXCEPT { return m_row; }

    MUDA_GENERIC size_t col() const MUDA_NOEXCEPT { return m_col; }

    MUDA_GENERIC size_t lda() const MUDA_NOEXCEPT
    {
        return m_view.pitch_bytes() / sizeof(Ty);
    }
};

template <typename Ty>
using DenseMatrixView = DenseMatrixViewT<false, Ty>;
template <typename Ty>
using CDenseMatrixView = DenseMatrixViewT<true, Ty>;
}  // namespace muda

#include "details/dense_matrix_view.inl"
