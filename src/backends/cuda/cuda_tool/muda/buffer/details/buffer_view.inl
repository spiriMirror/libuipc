#include <muda/buffer/buffer_launch.h>
#include <muda/compute_graph/compute_graph_builder.h>

namespace muda
{
template <bool IsConst, typename T>
MUDA_GENERIC BufferViewT<IsConst, T>::BufferViewT(auto_const_t<T>* data,
                                                  size_t           offset,
                                                  size_t size) MUDA_NOEXCEPT
    : m_data(data),
      m_offset(offset),
      m_size(size)
{
}

template <bool IsConst, typename T>
MUDA_GENERIC BufferViewT<IsConst, T>::BufferViewT(auto_const_t<T>* data, size_t size) MUDA_NOEXCEPT
    : m_data(data),
      m_offset(0),
      m_size(size)
{
}

template <bool IsConst, typename T>
template <bool OtherIsConst>
MUDA_GENERIC BufferViewT<IsConst, T>::BufferViewT(const BufferViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT
    MUDA_REQUIRES(!OtherIsConst)
    : m_data(other.m_data)
    , m_offset(other.m_offset)
    , m_size(other.m_size)
{
    static_assert(!OtherIsConst, "This must be non-const");
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::as_const() const MUDA_NOEXCEPT -> ConstView
{
    return ConstView{*this};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::data() const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    return m_data + m_offset;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::data(size_t i) const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    i += m_offset;
    return m_data + i;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::origin_data() const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    return m_data;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::operator[](size_t i) const MUDA_NOEXCEPT
    -> auto_const_t<T>&
{
    return *data(i);
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::subview(size_t offset, size_t size) const MUDA_NOEXCEPT
    -> ThisView
{
#ifndef __CUDA_ARCH__
    if(ComputeGraphBuilder::is_topo_building())
        return ThisView{};  // dummy
#endif
    if(size == ~0)
        size = m_size - offset;
    MUDA_KERNEL_ASSERT(offset + size <= m_size,
                       "BufferView out of range, size = %d, yours = %d",
                       m_size,
                       offset + size);
    return ThisView{m_data, m_offset + offset, size};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::viewer() const MUDA_NOEXCEPT -> ThisViewer
{
    return ThisViewer{data(), static_cast<int>(m_size)};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::cviewer() const MUDA_NOEXCEPT -> CViewer
{
    return CViewer{data(), static_cast<int>(m_size)};
}

template <bool IsConst, typename T>
MUDA_HOST void BufferViewT<IsConst, T>::fill(const T& v) const MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "This must be non-const");

    BufferLaunch().template fill<T>(*this, v);
}

template <bool IsConst, typename T>
MUDA_HOST void BufferViewT<IsConst, T>::copy_from(const BufferViewT<true, T>& other) const
    MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "This must be non-const");

    BufferLaunch().template copy<T>(*this, other);
}

template <bool IsConst, typename T>
MUDA_HOST void BufferViewT<IsConst, T>::copy_from(const T* host) const
    MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "This must be non-const");

    BufferLaunch()
        .template copy<T>(*this, host)  //
        .wait();
}

template <bool IsConst, typename T>
MUDA_HOST void BufferViewT<IsConst, T>::copy_to(T* host) const
{
    BufferLaunch()
        .template copy<T>(host, *this)  //
        .wait();
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::operator+(int i) const MUDA_NOEXCEPT -> ThisView
{
    return ThisView{m_data, m_offset + i, m_size - i};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::operator*() const MUDA_NOEXCEPT -> reference
{
    return *data(0);
}

template <bool IsConst, typename T>
MUDA_GENERIC auto BufferViewT<IsConst, T>::operator[](int i) const MUDA_NOEXCEPT
    -> auto_const_t<T>&
{
    return *data(i);
}
}  // namespace muda