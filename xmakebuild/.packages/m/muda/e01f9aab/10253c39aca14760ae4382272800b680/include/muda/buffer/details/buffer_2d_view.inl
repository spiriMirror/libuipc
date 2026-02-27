#include <muda/compute_graph/compute_graph_builder.h>
#include <muda/buffer/buffer_launch.h>

namespace muda
{
template <bool IsConst, typename T>
MUDA_GENERIC Buffer2DViewT<IsConst, T>::Buffer2DViewT(auto_const_t<T>* data,
                                                      size_t pitch_bytes,
                                                      size_t origin_width,
                                                      size_t origin_height,
                                                      const Offset2D& offset,
                                                      const Extent2D& extent) MUDA_NOEXCEPT
    : m_data(data),
      m_pitch_bytes(pitch_bytes),
      m_origin_width(origin_width),
      m_origin_height(origin_height),
      m_offset(offset),
      m_extent(extent)
{
}

template <bool IsConst, typename T>
template <bool OtherIsConst>
MUDA_GENERIC Buffer2DViewT<IsConst, T>::Buffer2DViewT(const Buffer2DViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT
    MUDA_REQUIRES(!OtherIsConst)
    : m_data(other.m_data)
    , m_pitch_bytes(other.m_pitch_bytes)
    , m_origin_width(other.m_origin_width)
    , m_origin_height(other.m_origin_height)
    , m_offset(other.m_offset)
    , m_extent(other.m_extent)
{
}

template <bool IsConst, typename T>
MUDA_GENERIC Buffer2DViewT<IsConst, T>::Buffer2DViewT(auto_const_t<T>* data,
                                                      size_t pitch_bytes,
                                                      const Offset2D& offset,
                                                      const Extent2D& extent) MUDA_NOEXCEPT
    : Buffer2DViewT(data, pitch_bytes, extent.width(), extent.height(), offset, extent)
{
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::as_const() const MUDA_NOEXCEPT -> ConstView
{
    return ConstView{*this};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::data(size_t x, size_t y) const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    x += m_offset.offset_in_height();
    y += m_offset.offset_in_width();

    auto height_begin =
        reinterpret_cast<std::byte*>(remove_const(m_data)) + m_pitch_bytes * x;
    return reinterpret_cast<auto_const_t<T>*>(height_begin) + y;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::data(size_t flatten_i) const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    auto x = flatten_i / m_extent.width();
    auto y = flatten_i % m_extent.width();
    return data(x, y);
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::origin_data() const MUDA_NOEXCEPT
    -> auto_const_t<T>*

{
    return m_data;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::subview(Offset2D offset, Extent2D extent) const MUDA_NOEXCEPT
    -> ThisView
{
#ifndef __CUDA_ARCH__
    if(ComputeGraphBuilder::is_topo_building())
        return ThisView{};  // dummy
#endif

    if(!extent.valid())
        extent = m_extent - as_extent(offset);

    MUDA_KERNEL_ASSERT(extent + as_extent(offset) <= m_extent,
                       "Buffer2DView out of range, extent = (%d,%d), yours = (%d,%d)",
                       (int)m_extent.height(),
                       (int)m_extent.width(),
                       (int)extent.height(),
                       (int)extent.width());
    return ThisView{m_data, m_pitch_bytes, m_origin_width, m_origin_height, offset, extent};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::viewer() const MUDA_NOEXCEPT -> ThisViewer
{
    return ThisViewer{m_data,
                      make_int2((int)m_offset.offset_in_height(),
                                (int)m_offset.offset_in_width()),
                      make_int2((int)m_extent.height(), (int)m_extent.width()),
                      (int)m_pitch_bytes};
}

template <bool IsConst, typename T>
MUDA_GENERIC cudaPitchedPtr Buffer2DViewT<IsConst, T>::cuda_pitched_ptr() const MUDA_NOEXCEPT
{
    // don't use make_cudaPitchedPtr (__host__ only function)
    return cudaPitchedPtr{.ptr   = remove_const(m_data),
                          .pitch = remove_const(m_pitch_bytes),
                          .xsize = m_origin_width * sizeof(T),
                          .ysize = m_origin_height};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::extent() const MUDA_NOEXCEPT -> Extent2D
{
    return m_extent;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::pitch_bytes() const MUDA_NOEXCEPT -> size_t
{
    return m_pitch_bytes;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::offset() const MUDA_NOEXCEPT -> Offset2D
{
    return m_offset;
}
template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::total_size() const MUDA_NOEXCEPT -> size_t
{
    return m_extent.width() * m_extent.height();
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer2DViewT<IsConst, T>::cviewer() const MUDA_NOEXCEPT -> CViewer
{
    return viewer();
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer2DViewT<IsConst, T>::copy_to(T* host) const
{
    BufferLaunch().template copy<T>(host, *this).wait();
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer2DViewT<IsConst, T>::fill(const T& val) MUDA_REQUIRES(!IsConst)
{
    BufferLaunch().template fill(*this, val);
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer2DViewT<IsConst, T>::copy_from(const Buffer2DViewT<true, T>& other)
    MUDA_REQUIRES(!IsConst)
{
    BufferLaunch().template copy<T>(*this, other);
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer2DViewT<IsConst, T>::copy_from(const T* host) MUDA_REQUIRES(!IsConst)
{
    BufferLaunch().template copy<T>(*this, host).wait();
}

}  // namespace muda