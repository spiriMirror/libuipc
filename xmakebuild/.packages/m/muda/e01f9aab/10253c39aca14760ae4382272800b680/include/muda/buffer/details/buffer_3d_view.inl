#include <muda/compute_graph/compute_graph_builder.h>
#include <muda/buffer/buffer_launch.h>

namespace muda
{
template <bool IsConst, typename T>
template <bool OtherIsConst>
MUDA_GENERIC Buffer3DViewT<IsConst, T>::Buffer3DViewT(const Buffer3DViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT
    : m_data(other.m_data),
      m_pitch_bytes(other.m_pitch_bytes),
      m_pitch_bytes_area(other.m_pitch_bytes_area),
      m_origin_width(other.m_origin_width),
      m_origin_height(other.m_origin_height),
      m_offset(other.m_offset),
      m_extent(other.m_extent)
{
}

template <bool IsConst, typename T>
MUDA_GENERIC Buffer3DViewT<IsConst, T>::Buffer3DViewT(auto_const_t<T>* data,
                                                      size_t pitch_bytes,
                                                      size_t pitch_bytes_area,
                                                      size_t origin_width,
                                                      size_t origin_height,
                                                      const Offset3D& offset,
                                                      const Extent3D& extent) MUDA_NOEXCEPT
    : m_data(data),
      m_pitch_bytes(pitch_bytes),
      m_pitch_bytes_area(pitch_bytes_area),
      m_origin_width(origin_width),
      m_origin_height(origin_height),
      m_offset(offset),
      m_extent(extent)
{
}

template <bool IsConst, typename T>
MUDA_GENERIC Buffer3DViewT<IsConst, T>::Buffer3DViewT(T*     data,
                                                      size_t pitch_bytes,
                                                      size_t pitch_bytes_area,
                                                      const Offset3D& offset,
                                                      const Extent3D& extent) MUDA_NOEXCEPT
    : Buffer3DViewT(data, pitch_bytes, pitch_bytes_area, extent.width(), extent.height(), offset, extent)
{
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::as_const() const MUDA_NOEXCEPT -> ConstView
{
    return ConstView{*this};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::data(size_t x, size_t y, size_t z) const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    x += m_offset.offset_in_depth();
    y += m_offset.offset_in_height();
    z += m_offset.offset_in_width();
    auto depth_begin =
        reinterpret_cast<std::byte*>(remove_const(m_data)) + m_pitch_bytes_area * x;
    auto height_begin = depth_begin + m_pitch_bytes * y;
    return reinterpret_cast<T*>(height_begin) + z;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::data(size_t flatten_i) const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    auto area       = m_extent.width() * m_extent.height();
    auto x          = flatten_i / area;
    auto i_in_area  = flatten_i % area;
    auto y          = i_in_area / m_extent.width();
    auto i_in_width = i_in_area % m_extent.width();
    auto z          = i_in_width;
    return data(x, y, z);
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::origin_data() const MUDA_NOEXCEPT
    -> auto_const_t<T>*
{
    return m_data;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::extent() const MUDA_NOEXCEPT -> Extent3D
{
    return m_extent;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::offset() const MUDA_NOEXCEPT -> Offset3D
{
    return m_offset;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::pitch_bytes() const MUDA_NOEXCEPT -> size_t
{
    return m_pitch_bytes;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::pitch_bytes_area() const MUDA_NOEXCEPT -> size_t
{
    return m_pitch_bytes_area;
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::cuda_pitched_ptr() const MUDA_NOEXCEPT
    -> cudaPitchedPtr
{
    // don't use make_cudaPitchedPtr (__host__ only function)
    return cudaPitchedPtr{.ptr   = remove_const(m_data),
                          .pitch = remove_const(m_pitch_bytes),
                          .xsize = m_origin_width * sizeof(T),
                          .ysize = m_origin_height};
}

template <bool IsConst, typename T>
MUDA_GENERIC size_t Buffer3DViewT<IsConst, T>::total_size() const MUDA_NOEXCEPT
{
    return m_extent.width() * m_extent.height() * m_extent.depth();
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::subview(Offset3D offset, Extent3D extent) const MUDA_NOEXCEPT
    -> ThisView
{
#ifndef __CUDA_ARCH__
    if(ComputeGraphBuilder::is_topo_building())
        return ThisView{};  // dummy
#endif

    if(!extent.valid())
        extent = m_extent - as_extent(offset);

    MUDA_KERNEL_ASSERT(extent + as_extent(offset) <= m_extent,
                       "Buffer3DView out of range, extent = (%d,%d,%d), yours = (%d,%d,%d)",
                       (int)m_extent.depth(),
                       (int)m_extent.height(),
                       (int)m_extent.width(),
                       (int)extent.depth(),
                       (int)extent.height(),
                       (int)extent.width());
    return ThisView{m_data, m_pitch_bytes, m_pitch_bytes_area, m_origin_width, m_origin_height, offset, extent};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::viewer() const MUDA_NOEXCEPT -> ThisViewer
{
    return ThisViewer{m_data,
                      make_int3(m_offset.offset_in_depth(),
                                m_offset.offset_in_height(),
                                m_offset.offset_in_width()),
                      make_int3(m_extent.depth(), m_extent.height(), m_extent.width()),
                      (int)m_pitch_bytes,
                      (int)m_pitch_bytes_area};
}

template <bool IsConst, typename T>
MUDA_GENERIC auto Buffer3DViewT<IsConst, T>::cviewer() const MUDA_NOEXCEPT -> CViewer
{
    return viewer();
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer3DViewT<IsConst, T>::fill(const T& v) const MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "This must be non-const buffer.");
    BufferLaunch().template fill<T>(*this, v);
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer3DViewT<IsConst, T>::copy_from(const Buffer3DViewT<true, T>& other) const
    MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "This must be non-const buffer.");
    BufferLaunch().template copy<T>(*this, other);
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer3DViewT<IsConst, T>::copy_from(const T* host) const
    MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "This must be non-const buffer.");
    BufferLaunch().template copy<T>(*this, host).wait();
}

template <bool IsConst, typename T>
MUDA_HOST void Buffer3DViewT<IsConst, T>::copy_to(T* host) const
{
    BufferLaunch().template copy<T>(host, *this).wait();
}
}  // namespace muda