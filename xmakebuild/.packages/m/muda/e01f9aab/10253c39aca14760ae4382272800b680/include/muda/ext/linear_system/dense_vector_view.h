#pragma once
#include <cusparse_v2.h>
#include <muda/ext/linear_system/dense_vector_viewer.h>
#include <muda/buffer/buffer_view.h>
#include <muda/view/view_base.h>
namespace muda
{
template <bool IsConst, typename T>
class DenseVectorViewT : public ViewBase<IsConst>
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                  "now only support real number");

    using Base = ViewBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    template <bool OtherIsConst, typename U>
    friend class DenseVectorViewT;

  public:
    using NonConstView = DenseVectorViewT<false, T>;
    using ConstView    = DenseVectorViewT<true, T>;
    using ThisView     = DenseVectorViewT<IsConst, T>;

    using CBufferView    = CBufferView<T>;
    using BufferView     = BufferView<T>;
    using ThisBufferView = std::conditional_t<IsConst, CBufferView, BufferView>;

    using CViewer    = CDenseVectorViewer<T>;
    using Viewer     = DenseVectorViewer<T>;
    using ThisViewer = std::conditional_t<IsConst, CViewer, Viewer>;

  protected:
    auto_const_t<T>*             m_data        = nullptr;
    mutable cusparseDnVecDescr_t m_descr       = nullptr;
    int                          m_offset      = -1;
    int                          m_inc         = -1;
    int                          m_size        = -1;
    int                          m_origin_size = -1;

  public:
    MUDA_GENERIC DenseVectorViewT() = default;

    MUDA_GENERIC DenseVectorViewT(auto_const_t<T>*     data,
                                  cusparseDnVecDescr_t descr,
                                  int                  offset,
                                  int                  inc,
                                  int                  size,
                                  int                  origin_size)
        : m_data(data)
        , m_descr(descr)
        , m_offset(offset)
        , m_inc(inc)
        , m_size(size)
        , m_origin_size(origin_size)
    {
    }

    template <bool OtherIsConst>
    MUDA_GENERIC DenseVectorViewT(const DenseVectorViewT<OtherIsConst, T>& other)
        MUDA_REQUIRES(IsConst)
        : m_data(other.m_data)
        , m_descr(other.m_descr)
        , m_offset(other.m_offset)
        , m_inc(other.m_inc)
        , m_size(other.m_size)
        , m_origin_size(other.m_origin_size)
    {
        static_assert(IsConst, "Cannot assign to a const viewer");
    }

    MUDA_GENERIC ConstView as_const() const
    {
        return ConstView{m_data, m_descr, m_offset, m_inc, m_size, m_origin_size};
    }

    // non-const accessor
    MUDA_GENERIC auto viewer() const
    {
        return ThisViewer{m_data, m_offset, m_size, m_origin_size};
    }
    MUDA_GENERIC auto buffer_view() const
    {
        return ThisBufferView{m_data, size_t(m_offset), size_t(m_inc * m_size)};
    }

    MUDA_GENERIC auto data() const { return m_data + m_offset; }

    MUDA_GENERIC auto origin_data() const { return m_data; }

    MUDA_GENERIC auto offset() const { return m_offset; }

    MUDA_GENERIC auto size() const { return m_size; }

    MUDA_GENERIC auto cviewer() const
    {
        MUDA_ASSERT(inc() == 1, "When using cviewer(), inc!=1 is not allowed");
        return CViewer{m_data, m_offset, m_size, m_origin_size};
    }

    MUDA_GENERIC auto inc() const { return m_inc; }

    MUDA_GENERIC auto descr() const
    {
        MUDA_ASSERT(inc() == 1, "When using descr(), inc!=1 is not allowed");
        return m_descr;
    }

    MUDA_GENERIC auto subview(int offset, int size) const
    {
        MUDA_ASSERT(inc() == 1, "When using subview(), inc!=1 is not allowed");
        MUDA_ASSERT(offset + size <= m_size, "subview out of range");
        return ThisView{m_data, m_descr, m_offset + offset, m_inc, size, m_origin_size};
    }
};

template <typename Ty>
using DenseVectorView = DenseVectorViewT<false, Ty>;
template <typename Ty>
using CDenseVectorView = DenseVectorViewT<true, Ty>;
}  // namespace muda

namespace muda
{
template <typename Ty>
struct read_only_view<DenseVectorView<Ty>>
{
    using type = CDenseVectorView<Ty>;
};

template <typename Ty>
struct read_write_view<DenseVectorView<Ty>>
{
    using type = DenseVectorView<Ty>;
};
}  // namespace muda


#include "details/dense_vector_view.inl"
