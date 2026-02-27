#include <muda/buffer/buffer_launch.h>

namespace muda
{
template <bool IsConst, typename T>
MUDA_GENERIC VarViewT<IsConst, T>::VarViewT(auto_const_t<T>* data) MUDA_NOEXCEPT
    : m_data(data)
{
}

template <bool IsConst, typename T>
template <bool OtherIsConst>
MUDA_GENERIC VarViewT<IsConst, T>::VarViewT(const VarViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT
    : m_data(other.m_data)
{
}

template <bool IsConst, typename T>
MUDA_GENERIC auto VarViewT<IsConst, T>::data() const MUDA_NOEXCEPT->auto_const_t<T>*
{
    return m_data;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto VarViewT<IsConst, T>::cviewer() const MUDA_NOEXCEPT->ConstViewer
{
    return ConstViewer{m_data};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto VarViewT<IsConst, T>::viewer() const MUDA_NOEXCEPT->ThisViewer
{
    return ThisViewer{m_data};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto VarViewT<IsConst, T>::as_const() const MUDA_NOEXCEPT->ConstView
{
    return ConstView{*this};
}

template <bool IsConst, typename T>
void VarViewT<IsConst, T>::copy_from(const T* val) const MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "Cannot copy to const var");

    BufferLaunch()
        .template copy<T>(*this, val)  //
        .wait();
}

template <bool IsConst, typename T>
void VarViewT<IsConst, T>::copy_to(T* val) const
{
    BufferLaunch()
        .template copy<T>(val, *this)  //
        .wait();
}

template <bool IsConst, typename T>
void VarViewT<IsConst, T>::copy_from(const ConstView& val) const MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "Cannot copy to const var");

    BufferLaunch()
        .template copy<T>(*this, val)  //
        .wait();
}

template <bool IsConst, typename T>
void VarViewT<IsConst, T>::fill(const T& val) const MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "Cannot fill const var");

    BufferLaunch()
        .template fill<T>(*this, val)  //
        .wait();
}
}  // namespace muda