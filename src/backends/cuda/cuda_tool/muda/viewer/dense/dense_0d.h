#pragma once
#include <muda/viewer/viewer_base.h>

namespace muda
{
template <bool IsConst, typename T>
class DenseViewerT : public ViewerBase<IsConst>
{
    using Base = ViewerBase<IsConst>;

    template <bool OtherIsConst, typename U>
    friend class DenseViewerT;

    MUDA_VIEWER_COMMON_NAME(DenseViewerT);

  public:
    using ConstViewer    = DenseViewerT<true, T>;
    using NonConstViewer = DenseViewerT<false, T>;
    using ThisViewer     = DenseViewerT<IsConst, T>;

  protected:
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    auto_const_t<T>* m_data = nullptr;

  public:
    using value_type = T;

    MUDA_GENERIC DenseViewerT() MUDA_NOEXCEPT = default;

    MUDA_GENERIC explicit DenseViewerT(auto_const_t<T>* p) MUDA_NOEXCEPT : m_data(p)
    {
    }

    MUDA_GENERIC DenseViewerT(const DenseViewerT&) MUDA_NOEXCEPT = default;

    template <bool OtherIsConst>
    MUDA_GENERIC DenseViewerT(const DenseViewerT<OtherIsConst, T>& other) MUDA_NOEXCEPT
        : m_data(other.m_data)
    {
    }

    MUDA_GENERIC ThisViewer& operator=(const T& v) MUDA_NOEXCEPT MUDA_REQUIRES(!IsConst)
    {
        static_assert(!IsConst, "Cannot assign to a const viewer");
        check();
        *m_data = v;
        return *this;
    }

    MUDA_GENERIC auto as_const() const MUDA_NOEXCEPT
    {
        return ConstViewer{m_data};
    }

    MUDA_GENERIC auto_const_t<T>& operator*() const MUDA_NOEXCEPT
    {
        check();
        return *m_data;
    }

    MUDA_GENERIC auto_const_t<T>* operator->() const MUDA_NOEXCEPT
    {
        check();
        return m_data;
    }

    MUDA_GENERIC auto_const_t<T>* data() const MUDA_NOEXCEPT { return m_data; }

    MUDA_GENERIC operator auto_const_t<T>&() const MUDA_NOEXCEPT
    {
        check();
        return *m_data;
    }

  protected:
    MUDA_INLINE MUDA_GENERIC void check() const MUDA_NOEXCEPT
    {
        if constexpr(DEBUG_VIEWER)
        {
            MUDA_KERNEL_ASSERT(m_data,
                               "Dense[%s:%s]: m_data is null. %s(%d)",
                               this->name(),
                               this->kernel_name(),
                               this->kernel_file(),
                               this->kernel_line());
        }
    }
};

template <typename T>
using Dense = DenseViewerT<false, T>;

template <typename T>
using CDense = DenseViewerT<true, T>;

// viewer traits
template <typename T>
struct read_only_view<Dense<T>>
{
    using type = CDense<T>;
};

template <typename T>
struct read_write_view<CDense<T>>
{
    using type = Dense<T>;
};

// make functions
template <typename T>
MUDA_INLINE MUDA_GENERIC auto make_cdense(const T* data) MUDA_NOEXCEPT
{
    return CDense<T>{data};
}

template <typename T>
MUDA_INLINE MUDA_GENERIC auto make_dense(T* data) MUDA_NOEXCEPT
{
    return Dense<T>{data};
}

//print convert
template <typename T>
MUDA_INLINE MUDA_GENERIC const T& print_convert(const Dense<T>& v) MUDA_NOEXCEPT
{
    return v.operator T&();
}

template <typename T>
MUDA_INLINE MUDA_GENERIC const T& print_convert(const CDense<T>& v) MUDA_NOEXCEPT
{
    return v.operator const T&();
}

}  // namespace muda
