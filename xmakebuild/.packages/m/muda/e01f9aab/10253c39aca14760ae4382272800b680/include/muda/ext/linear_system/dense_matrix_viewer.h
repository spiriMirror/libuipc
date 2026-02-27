#pragma once
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <muda/buffer/buffer_2d_view.h>
#include <muda/viewer/viewer_base.h>
#include <muda/atomic.h>

namespace muda
{
template <bool IsConst, typename T>
class DenseMatrixViewerT : public ViewerBase<IsConst>
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                  "now only support real number");
    static_assert(!std::is_const_v<T>, "T must be non-const type");

    using Base = ViewerBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U>
    friend class DenseMatrixViewerT;


  public:
    using CBuffer2DView = CBuffer2DView<T>;
    using Buffer2DView  = Buffer2DView<T>;
    using ThisBuffer2DView = std::conditional_t<IsConst, CBuffer2DView, Buffer2DView>;

    using ConstViewer    = DenseMatrixViewerT<true, T>;
    using NonConstViewer = DenseMatrixViewerT<false, T>;
    using ThisViewer = std::conditional_t<IsConst, ConstViewer, NonConstViewer>;

    using MatrixType = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
    template <typename U>
    using MapMatrixT =
        Eigen::Map<U, Eigen::AlignmentType::Unaligned, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>;
    using MapMatrix     = MapMatrixT<MatrixType>;
    using CMapMatrix    = MapMatrixT<const MatrixType>;
    using ThisMapMatrix = std::conditional_t<IsConst, CMapMatrix, MapMatrix>;

    MUDA_VIEWER_COMMON_NAME(DenseMatrixViewerT);

  protected:
    ThisBuffer2DView m_view;
    size_t           m_row_offset = 0;
    size_t           m_col_offset = 0;
    size_t           m_row_size   = 0;
    size_t           m_col_size   = 0;

  public:
    MUDA_GENERIC DenseMatrixViewerT(ThisBuffer2DView view,
                                    size_t           row_offset,
                                    size_t           col_offset,
                                    size_t           row_size,
                                    size_t           col_size)
        : m_view(view)
        , m_row_offset(row_offset)
        , m_col_offset(col_offset)
        , m_row_size(row_size)
        , m_col_size(col_size)
    {
    }

    template <bool OtherIsConst>
    MUDA_GENERIC DenseMatrixViewerT(const DenseMatrixViewerT<OtherIsConst, T>& other)
        MUDA_REQUIRES(IsConst)
        : m_view(other.m_view)
        , m_row_offset(other.m_row_offset)
        , m_col_offset(other.m_col_offset)
        , m_row_size(other.m_row_size)
        , m_col_size(other.m_col_size)
    {
        static_assert(IsConst);
    }

    MUDA_GENERIC auto as_const() const
    {
        return ConstViewer{m_view, m_row_offset, m_col_offset, m_row_size, m_col_size};
    }

    MUDA_GENERIC ThisViewer block(size_t row_offset, size_t col_offset, size_t row_size, size_t col_size) const
    {
        MUDA_ASSERT(row_offset + row_size <= m_row_size && col_offset + col_size <= m_col_size,
                    "DenseMatrixViewerBase [%s:%s]: block index out of range, shape=(%lld,%lld), yours index=(%lld,%lld). %s(%d)",
                    this->name(),
                    this->kernel_name(),
                    m_row_size,
                    m_col_size,
                    row_offset,
                    col_offset,
                    this->kernel_file(),
                    this->kernel_line());

        auto ret = DenseMatrixViewerT{
            m_view, m_row_offset + row_offset, m_col_offset + col_offset, row_size, col_size};
        ret.copy_label(*this);
        return ret;
    }

    template <int M, int N>
    MUDA_GENERIC ThisViewer block(int row_offset, int col_offset)
    {
        return block(row_offset, col_offset, M, N);
    }
    MUDA_GENERIC Eigen::Block<ThisMapMatrix> as_eigen() const
    {
        auto outer = m_view.pitch_bytes() / sizeof(T);

        return ThisMapMatrix{m_view.origin_data(),
                             (int)origin_row(),
                             (int)origin_col(),
                             Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>{(int)outer, 1}}
            .block(m_row_offset, m_col_offset, m_row_size, m_col_size);
    }

    MUDA_GENERIC auto_const_t<T>& operator()(size_t i, size_t j) const
    {
        if constexpr(DEBUG_VIEWER)
        {
            MUDA_ASSERT(m_view.data(0),
                        "DenseMatrixViewer [%s:%s]: data is null",
                        this->name(),
                        this->kernel_name());
            if(m_row_offset == 0 && m_col_offset == 0)
            {
                MUDA_ASSERT(i < m_row_size && j < m_col_size,
                            "DenseMatrixViewer [%s:%s]: index out of range, shape=(%lld,%lld), yours index=(%lld,%lld)",
                            this->name(),
                            this->kernel_name(),
                            m_row_size,
                            m_col_size,
                            i,
                            j);
            }
            else
            {
                MUDA_ASSERT(i < m_row_size && j < m_col_size,
                            "DenseMatrixViewer [%s:%s]:index out of range, block shape=(%lld,%lld), your index=(%lld,%lld)",
                            this->name(),
                            this->kernel_name(),
                            m_row_size,
                            m_col_size,
                            i,
                            j);
            }
        }
        i += m_row_offset;
        j += m_col_offset;
        return *m_view.data(j, i);
    }

    MUDA_GENERIC auto buffer_view() const { return m_view; }

    MUDA_GENERIC size_t row() const { return m_row_size; }

    MUDA_GENERIC size_t col() const { return m_col_size; }

    MUDA_GENERIC size_t origin_row() const
    {
        size_t ret;
        ret = m_view.extent().width();
        return ret;
    }

    MUDA_GENERIC size_t origin_col() const
    {
        size_t ret;
        ret = m_view.extent().height();
        return ret;
    }

    MUDA_GENERIC auto row_offset() const { return m_row_offset; }

    MUDA_GENERIC auto col_offset() const { return m_col_offset; }

    MUDA_DEVICE T atomic_add(size_t i, size_t j, T val) const MUDA_REQUIRES(!IsConst)
    {
        static_assert(!IsConst, "Cannot assign to a const viewer");
        auto ptr = &this->operator()(i, j);
        muda::atomic_add(ptr, val);
        return val;
    }

    template <int M, int N>
    MUDA_DEVICE Eigen::Matrix<T, M, N> atomic_add(const Eigen::Matrix<T, M, N>& other) const
        MUDA_REQUIRES(!IsConst)
    {
        static_assert(!IsConst, "Cannot assign to a const viewer");
        check_size_matching(M, N);
        Eigen::Matrix<T, M, N> ret;
#pragma unroll
        for(int i = 0; i < M; ++i)
#pragma unroll
            for(int j = 0; j < N; ++j)
            {
                ret(i, j) = atomic_add(i, j, other(i, j));
            }
        return ret;
    }

    template <int M, int N>
    MUDA_GENERIC DenseMatrixViewerT& operator=(const Eigen::Matrix<T, M, N>& other) const
        MUDA_REQUIRES(!IsConst)
    {
        static_assert(!IsConst, "Cannot assign to a const viewer");
        check_size_matching(M, N);
#pragma unroll
        for(int i = 0; i < M; ++i)
#pragma unroll
            for(int j = 0; j < N; ++j)
                (*this)(i, j) = other(i, j);
        return *this;
    }

  private:
    MUDA_GENERIC void check_size_matching(int M, int N) const
    {
        MUDA_ASSERT(this->m_row_size == M && this->m_col_size == N,
                    "DenseMatrixViewer [%s:%s] shape mismatching, Viewer=(%lld,%lld), yours=(%lld,%lld). %s(%d)",
                    this->name(),
                    this->kernel_name(),
                    this->m_row_size,
                    this->m_col_size,
                    M,
                    N,
                    this->kernel_file(),
                    this->kernel_line());
    }
};

template <typename T>
using DenseMatrixViewer = DenseMatrixViewerT<false, T>;

template <typename T>
using CDenseMatrixViewer = DenseMatrixViewerT<true, T>;
}  // namespace muda

#include "details/dense_matrix_viewer.inl"
