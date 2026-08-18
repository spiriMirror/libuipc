#pragma once
#include <muda/ext/linear_system/doublet_vector_view.h>
#include <muda/ext/linear_system/bcoo_vector_viewer.h>

namespace muda
{
template <typename T, int N>
using BCOOVectorView = DoubletVectorView<T, N>;
template <typename T, int N>
using CBCOOVectorView = CDoubletVectorView<T, N>;
}  // namespace muda

namespace muda
{
template <bool IsConst, typename T>
class COOVectorViewBase : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

  public:
    static_assert(!std::is_const_v<T>, "T must be non-const");
    using NonConstView = COOVectorViewBase<false, T>;
    using ConstView    = COOVectorViewBase<true, T>;
    using ThisView     = COOVectorViewBase<IsConst, T>;

    using CViewer    = CCOOVectorViewer<T>;
    using Viewer     = COOVectorViewer<T>;
    using ThisViewer = std::conditional_t<IsConst, CViewer, Viewer>;

  protected:
    // vector info
    int m_size = 0;

    //doublet info
    int m_doublet_index_offset = 0;
    int m_doublet_count        = 0;
    int m_total_doublet_count  = 0;

    // data
    auto_const_t<int>* m_indices = nullptr;
    auto_const_t<T>*   m_values  = nullptr;

    mutable cusparseSpVecDescr_t m_descr = nullptr;

  public:
    MUDA_GENERIC COOVectorViewBase() = default;
    MUDA_GENERIC COOVectorViewBase(int                  size,
                                   int                  doublet_index_offset,
                                   int                  doublet_count,
                                   int                  total_doublet_count,
                                   auto_const_t<int>*   indices,
                                   auto_const_t<T>*     values,
                                   cusparseSpVecDescr_t descr)
        : m_size(size)
        , m_doublet_index_offset(doublet_index_offset)
        , m_doublet_count(doublet_count)
        , m_total_doublet_count(total_doublet_count)
        , m_indices(indices)
        , m_values(values)
        , m_descr(descr)
    {
        MUDA_KERNEL_ASSERT(doublet_index_offset + doublet_count <= total_doublet_count,
                           "COOVectorView: out of range, m_total_doublet_count=%d, "
                           "your doublet_index_offset=%d, doublet_count=%d",
                           total_doublet_count,
                           doublet_index_offset,
                           doublet_count);
    }

    // implicit conversion

    MUDA_GENERIC auto as_const() const -> ConstView
    {
        return ConstView{m_size,
                         m_doublet_index_offset,
                         m_doublet_count,
                         m_total_doublet_count,
                         m_indices,
                         m_values,
                         m_descr};
    }

    // non-const accessor

    MUDA_GENERIC auto viewer()
    {
        return ThisViewer{
            m_size, m_doublet_index_offset, m_doublet_count, m_total_doublet_count, m_indices, m_values};
    }

    MUDA_GENERIC auto subview(int offset, int count)
    {
        return ThisView{m_size,
                        m_doublet_index_offset + offset,
                        count,
                        m_total_doublet_count,
                        m_indices,
                        m_values,
                        m_descr};
    }

    MUDA_GENERIC auto subview(int offset)
    {
        return subview(offset, m_doublet_count - offset);
    }

    // const accessor

    MUDA_GENERIC ConstView subview(int offset, int count) const
    {
        return remove_const(*this).subview(offset, count);
    }

    MUDA_GENERIC ConstView subview(int offset) const
    {
        return remove_const(*this).subview(offset);
    }

    MUDA_GENERIC auto cviewer() const { return remove_const(*this).viewer(); }


    MUDA_GENERIC auto vector_size() const { return m_size; }

    MUDA_GENERIC auto doublet_index_offset() const
    {
        return m_doublet_index_offset;
    }

    MUDA_GENERIC auto doublet_count() const { return m_doublet_count; }

    MUDA_GENERIC auto total_doublet_count() const
    {
        return m_total_doublet_count;
    }

    MUDA_GENERIC auto descr() const { return m_descr; }
};

template <typename T>
using COOVectorView = COOVectorViewBase<false, T>;
template <typename T>
using CCOOVectorView = COOVectorViewBase<true, T>;
}  // namespace muda

namespace muda
{
template <typename T>
struct read_only_view<COOVectorView<T>>
{
    using type = CCOOVectorView<T>;
};

template <typename T>
struct read_write_view<CCOOVectorView<T>>
{
    using type = COOVectorView<T>;
};
}  // namespace muda

#include "details/bcoo_vector_view.inl"
