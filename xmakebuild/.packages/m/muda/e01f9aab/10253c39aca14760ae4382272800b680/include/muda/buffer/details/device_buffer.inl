#include <muda/container/vector.h>
#include <muda/buffer/buffer_launch.h>

namespace muda
{
template <typename T>
DeviceBuffer<T>::DeviceBuffer(size_t n)
{
    BufferLaunch().alloc(*this, n);
}

template <typename T>
DeviceBuffer<T>::DeviceBuffer()
    : m_data(nullptr)
    , m_size(0)
    , m_capacity(0)
{
}

template <typename T>
DeviceBuffer<T>::DeviceBuffer(const DeviceBuffer<T>& other)
{
    BufferLaunch()
        .alloc(*this, other.size())  //
        .copy(view(), other.view());
}

template <typename T>
DeviceBuffer<T>::DeviceBuffer(DeviceBuffer<T>&& other) MUDA_NOEXCEPT
    : m_data(other.m_data),
      m_size(other.m_size),
      m_capacity(other.m_capacity)
{
    other.m_data     = nullptr;
    other.m_size     = 0;
    other.m_capacity = 0;
}

template <typename T>
DeviceBuffer<T>& DeviceBuffer<T>::operator=(const DeviceBuffer<T>& other)
{
    if(this == &other)
        return *this;

    BufferLaunch()
        .resize(*this, other.size())  //
        .copy(view(), other.view());

    return *this;
}

template <typename T>
DeviceBuffer<T>& DeviceBuffer<T>::operator=(DeviceBuffer<T>&& other)
{
    if(this == &other)
        return *this;

    if(m_data)
    {
        BufferLaunch().free(*this);
    }

    m_data     = other.m_data;
    m_size     = other.m_size;
    m_capacity = other.m_capacity;

    other.m_data     = nullptr;
    other.m_size     = 0;
    other.m_capacity = 0;

    return *this;
}

template <typename T>
DeviceBuffer<T>::DeviceBuffer(CBufferView<T> other)
{
    BufferLaunch()
        .alloc(*this, other.size())  //
        .copy(view(), other);
}

template <typename T>
DeviceBuffer<T>::DeviceBuffer(const std::vector<T>& host)
{
    BufferLaunch()
        .alloc(*this, host.size())  //
        .copy(view(), host.data())  //
        .wait();
}

template <typename T>
DeviceBuffer<T>& DeviceBuffer<T>::operator=(CBufferView<T> other)
{
    BufferLaunch()
        .resize(*this, other.size())  //
        .copy(view(), other);
    return *this;
}

template <typename T>
DeviceBuffer<T>& DeviceBuffer<T>::operator=(const std::vector<T>& other)
{
    BufferLaunch()
        .resize(*this, other.size())  //
        .copy(view(), other.data())   //
        .wait();
    return *this;
}

template <typename T>
template <typename Alloc>
void DeviceBuffer<T>::copy_to(std::vector<T, Alloc>& host) const
{
    host.resize(size());
    view().copy_to(host.data());
}

template <typename T>
template <typename Alloc>
void DeviceBuffer<T>::copy_from(const std::vector<T, Alloc>& host)
{
    resize(host.size());
    view().copy_from(host.data());
}

template <typename T>
void DeviceBuffer<T>::resize(size_t new_size)
{
    BufferLaunch().resize(*this, new_size);
}

template <typename T>
void DeviceBuffer<T>::resize(size_t new_size, const value_type& value)
{
    BufferLaunch().resize(*this, new_size, value);
}

template <typename T>
void DeviceBuffer<T>::reserve(size_t new_capacity)
{
    BufferLaunch().reserve(*this, new_capacity);
}

template <typename T>
void DeviceBuffer<T>::clear()
{
    BufferLaunch().clear(*this);
}

template <typename T>
void DeviceBuffer<T>::shrink_to_fit()
{
    BufferLaunch().shrink_to_fit(*this);
}

template <typename T>
void DeviceBuffer<T>::fill(const T& v)
{
    view().fill(v);
};

template <typename T>
Dense1D<T> DeviceBuffer<T>::viewer() MUDA_NOEXCEPT
{
    return view().viewer();
}

template <typename T>
CDense1D<T> DeviceBuffer<T>::cviewer() const MUDA_NOEXCEPT
{
    return view().cviewer();
}

template <typename T>
BufferView<T> DeviceBuffer<T>::view(size_t offset, size_t size) MUDA_NOEXCEPT
{
    return view().subview(offset, size);
}

template <typename T>
BufferView<T> DeviceBuffer<T>::view() MUDA_NOEXCEPT
{
    return BufferView<T>{m_data, 0, m_size};
}

template <typename T>
CBufferView<T> DeviceBuffer<T>::view(size_t offset, size_t size) const MUDA_NOEXCEPT
{
    return view().subview(offset, size);
}

template <typename T>
CBufferView<T> DeviceBuffer<T>::view() const MUDA_NOEXCEPT
{
    return CBufferView<T>{m_data, 0, m_size};
}

template <typename T>
DeviceBuffer<T>::~DeviceBuffer()
{
    if(m_data)
    {
        BufferLaunch().free(*this);
    }
}
}  // namespace muda