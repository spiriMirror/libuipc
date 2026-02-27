#include <muda/buffer/buffer_launch.h>

namespace muda
{
template <typename T>
DeviceBuffer3D<T>::DeviceBuffer3D(const Extent3D& n)
{
    BufferLaunch().resize(*this, n);
}

template <typename T>
DeviceBuffer3D<T>::DeviceBuffer3D()
    : m_data(nullptr)
    , m_pitch_bytes(0)
    , m_pitch_bytes_area(0)
    , m_extent(Extent3D::Zero())
    , m_capacity(Extent3D::Zero())
{
}

template <typename T>
DeviceBuffer3D<T>::DeviceBuffer3D(const DeviceBuffer3D<T>& other)
{
    BufferLaunch()
        .resize(*this, other.extent())  //
        .copy(view(), other.view());
}

template <typename T>
DeviceBuffer3D<T>::DeviceBuffer3D(DeviceBuffer3D<T>&& other) MUDA_NOEXCEPT
    : m_data(other.m_data),
      m_pitch_bytes(other.m_pitch_bytes),
      m_pitch_bytes_area(other.m_pitch_bytes_area),
      m_extent(other.m_extent),
      m_capacity(other.m_capacity)
{
    other.m_data             = nullptr;
    other.m_pitch_bytes      = 0;
    other.m_pitch_bytes_area = 0;
    other.m_extent           = Extent3D::Zero();
    other.m_capacity         = Extent3D::Zero();
}

template <typename T>
DeviceBuffer3D<T>& DeviceBuffer3D<T>::operator=(const DeviceBuffer3D<T>& other)
{
    if(this == &other)
        return *this;

    BufferLaunch()
        .resize(*this, other.extent())  //
        .copy(view(), other.view());

    return *this;
}

template <typename T>
DeviceBuffer3D<T>& DeviceBuffer3D<T>::operator=(DeviceBuffer3D<T>&& other)
{
    if(this == &other)
        return *this;

    if(m_data)
        BufferLaunch().free(*this).wait();

    m_data             = other.m_data;
    m_pitch_bytes      = other.m_pitch_bytes;
    m_pitch_bytes_area = other.m_pitch_bytes_area;
    m_extent           = other.m_extent;
    m_capacity         = other.m_capacity;

    other.m_data             = nullptr;
    other.m_pitch_bytes      = 0;
    other.m_pitch_bytes_area = 0;
    other.m_extent           = Extent3D::Zero();
    other.m_capacity         = Extent3D::Zero();

    return *this;
}

template <typename T>
DeviceBuffer3D<T>::DeviceBuffer3D(CBuffer3DView<T> other)
{
    BufferLaunch()
        .alloc(*this, other.extent())  //
        .copy(view(), other);
}

template <typename T>
DeviceBuffer3D<T>& DeviceBuffer3D<T>::operator=(CBuffer3DView<T> other)
{
    BufferLaunch()
        .resize(*this, other.extent())  //
        .copy(view(), other);

    return *this;
}

template <typename T>
template <typename Alloc>
void DeviceBuffer3D<T>::copy_to(std::vector<T, Alloc>& host) const
{
    host.resize(total_size());
    view().copy_to(host.data());
}

template <typename T>
template <typename Alloc>
void DeviceBuffer3D<T>::copy_from(const std::vector<T, Alloc>& host)
{
    MUDA_ASSERT(host.size() == total_size(),
                "Need eqaul total size, host_size=%d, total_size=%d",
                host.size(),
                total_size());

    view().copy_from(host.data());
}

template <typename T>
void DeviceBuffer3D<T>::resize(Extent3D new_extent)
{
    BufferLaunch().resize(*this, new_extent);
}

template <typename T>
void DeviceBuffer3D<T>::resize(Extent3D new_extent, const T& value)
{
    BufferLaunch().resize(*this, new_extent, value);
}

template <typename T>
void DeviceBuffer3D<T>::reserve(Extent3D new_capacity)
{
    BufferLaunch().reserve(*this, new_capacity);
}

template <typename T>
void DeviceBuffer3D<T>::clear()
{
    BufferLaunch().clear(*this);
}

template <typename T>
void DeviceBuffer3D<T>::shrink_to_fit()
{
    BufferLaunch().shrink_to_fit(*this);
}

template <typename T>
void DeviceBuffer3D<T>::fill(const T& v)
{
    BufferLaunch().fill(view(), v);
}

template <typename T>
DeviceBuffer3D<T>::~DeviceBuffer3D()
{
    if(m_data)
        BufferLaunch().free(*this);
}
}  // namespace muda