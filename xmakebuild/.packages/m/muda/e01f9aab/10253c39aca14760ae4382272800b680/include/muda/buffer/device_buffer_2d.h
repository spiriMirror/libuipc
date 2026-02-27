#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <vector>
#include <muda/viewer/dense.h>
#include <muda/buffer/buffer_2d_view.h>

namespace muda
{
template <typename T>
class DeviceBuffer2D
{
  private:
    friend class BufferLaunch;
    friend class NDReshaper;
    T*       m_data        = nullptr;
    size_t   m_pitch_bytes = 0;
    Extent2D m_extent      = Extent2D::Zero();
    Extent2D m_capacity    = Extent2D::Zero();

  public:
    using value_type = T;

    DeviceBuffer2D(const Extent2D& n);
    DeviceBuffer2D();

    DeviceBuffer2D(const DeviceBuffer2D<T>& other);
    DeviceBuffer2D(DeviceBuffer2D&& other) MUDA_NOEXCEPT;
    DeviceBuffer2D& operator=(const DeviceBuffer2D<T>& other);
    DeviceBuffer2D& operator=(DeviceBuffer2D<T>&& other);

    DeviceBuffer2D(CBuffer2DView<T> other);
    DeviceBuffer2D& operator=(CBuffer2DView<T> other);

    template <typename Alloc>
    void copy_to(std::vector<T, Alloc>& host) const;
    template <typename Alloc>
    void copy_from(const std::vector<T, Alloc>& host);

    void resize(Extent2D new_extent);
    void resize(Extent2D new_extent, const T& value);
    void reserve(Extent2D new_capacity);
    void clear();
    void shrink_to_fit();
    void fill(const T& v);

    Dense2D<T>  viewer() MUDA_NOEXCEPT { return view().viewer(); }
    CDense2D<T> cviewer() const MUDA_NOEXCEPT { return view().viewer(); }

    Buffer2DView<T> view(Offset2D offset, Extent2D extent = {}) MUDA_NOEXCEPT
    {
        return view().subview(offset, extent);
    }
    Buffer2DView<T> view() MUDA_NOEXCEPT
    {
        return Buffer2DView<T>{m_data, m_pitch_bytes, Offset2D::Zero(), m_extent};
    }
    operator Buffer2DView<T>() MUDA_NOEXCEPT { return view(); }

    CBuffer2DView<T> view(Offset2D offset, Extent2D extent = {}) const MUDA_NOEXCEPT
    {
        return view().subview(offset, extent);
    }

    CBuffer2DView<T> view() const MUDA_NOEXCEPT
    {
        return CBuffer2DView<T>{m_data, m_pitch_bytes, Offset2D::Zero(), m_extent};
    }
    operator CBuffer2DView<T>() const MUDA_NOEXCEPT { return view(); }

    ~DeviceBuffer2D();

    auto extent() const MUDA_NOEXCEPT { return m_extent; }
    auto capacity() const MUDA_NOEXCEPT { return m_capacity; }
    auto pitch_bytes() const MUDA_NOEXCEPT { return m_pitch_bytes; }
    auto total_size() const MUDA_NOEXCEPT
    {
        return m_extent.width() * m_extent.height();
    }
    T*       data() MUDA_NOEXCEPT { return m_data; }
    const T* data() const MUDA_NOEXCEPT { return m_data; }
};
}  // namespace muda

#include "details/device_buffer_2d.inl"