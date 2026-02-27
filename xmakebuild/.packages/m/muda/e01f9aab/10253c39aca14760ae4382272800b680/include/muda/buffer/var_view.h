#pragma once
#include <cuda.h>
#include <cinttypes>
#include <muda/view/view_base.h>
#include <muda/viewer/dense.h>

namespace muda
{
template <bool IsConst, typename T>
class VarViewT : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;

    template <bool OtherIsConst, typename U>
    friend class VarViewT;

  protected:
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    auto_const_t<T>* m_data = nullptr;

  public:
    using ConstView = VarViewT<true, T>;
    using ThisView  = VarViewT<IsConst, T>;

    using ConstViewer    = CDense<T>;
    using NonConstViewer = Dense<T>;
    using ThisViewer = typename std::conditional_t<IsConst, ConstViewer, NonConstViewer>;

    MUDA_GENERIC VarViewT() MUDA_NOEXCEPT = default;
    MUDA_GENERIC VarViewT(auto_const_t<T>* data) MUDA_NOEXCEPT;

    MUDA_GENERIC VarViewT(const VarViewT& other) MUDA_NOEXCEPT = default;
    template <bool OtherIsConst>
    MUDA_GENERIC VarViewT(const VarViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* data() const MUDA_NOEXCEPT;

    MUDA_GENERIC ConstView as_const() const MUDA_NOEXCEPT;

    MUDA_GENERIC ConstViewer cviewer() const MUDA_NOEXCEPT;
    MUDA_GENERIC ThisViewer  viewer() const MUDA_NOEXCEPT;

    MUDA_HOST void fill(const T& value) const MUDA_REQUIRES(!IsConst);
    MUDA_HOST void copy_to(T* data) const;
    MUDA_HOST void copy_from(const T* data) const MUDA_REQUIRES(!IsConst);
    MUDA_HOST void copy_from(const ConstView& val) const MUDA_REQUIRES(!IsConst);

    /**********************************************************************************
    * VarView As Iterator
    ***********************************************************************************/

    // Random Access Iterator Interface
    using value_type        = T;
    using reference         = T&;
    using pointer           = T*;
    using iterator_category = std::random_access_iterator_tag;
    using difference_type   = size_t;

    MUDA_GENERIC reference operator*() { return *data(); }
    MUDA_GENERIC auto_const_t<T>& operator[](int i) const { return *data(); }
};

template <typename T>
using VarView = VarViewT<false, T>;

template <typename T>
using CVarView = VarViewT<true, T>;

// viewer traits
template <typename T>
struct read_only_view<VarView<T>>
{
    using type = CVarView<T>;
};

template <typename T>
struct read_write_view<CVarView<T>>
{
    using type = VarView<T>;
};
}  // namespace muda


#include "details/var_view.inl"