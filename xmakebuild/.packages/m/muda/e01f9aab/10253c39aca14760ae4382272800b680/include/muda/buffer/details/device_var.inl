#include <muda/buffer/buffer_launch.h>
#include <muda/launch/memory.h>

namespace muda
{
template <typename T>
DeviceVar<T>::DeviceVar()
{
    Memory().alloc(&m_data, sizeof(T)).wait();
}
template <typename T>
DeviceVar<T>::DeviceVar(const T& value)
{
    Memory().alloc(&m_data, sizeof(T));
    view().copy_from(&value);
};

template <typename T>
DeviceVar<T>::DeviceVar(const DeviceVar& other)
{
    Memory().alloc(&m_data, sizeof(T)).wait();
    view().copy_from(other.view());
}

template <typename T>
DeviceVar<T>& DeviceVar<T>::operator=(const DeviceVar<T>& other)
{
    if(this == &other)
        return *this;
    view().copy_from(other.view());
    return *this;
}

template <typename T>
DeviceVar<T>& DeviceVar<T>::operator=(DeviceVar<T>&& other)
{
    if(this == &other)
        return *this;

    if(m_data)
        Memory().free(m_data).wait();

    m_data = other.m_data;

    other.m_data = nullptr;

    return *this;
}

template <typename T>
DeviceVar<T>& DeviceVar<T>::operator=(CVarView<T> other)
{
    view().copy_from(other);
    return *this;
}

template <typename T>
void DeviceVar<T>::copy_from(CVarView<T> other)
{
    view().copy_from(other);
}

template <typename T>
DeviceVar<T>& DeviceVar<T>::operator=(const T& val)
{
    view().copy_from(&val);
    return *this;
}

template <typename T>
DeviceVar<T>::DeviceVar(DeviceVar&& other) MUDA_NOEXCEPT : m_data(other.m_data)
{
    other.m_data = nullptr;
}

template <typename T>
DeviceVar<T>::operator T() const
{
    T var;
    view().copy_to(&var);
    return var;
}

template <typename T>
Dense<T> DeviceVar<T>::viewer() MUDA_NOEXCEPT
{
    return Dense<T>(m_data);
}

template <typename T>
CDense<T> DeviceVar<T>::cviewer() const MUDA_NOEXCEPT
{
    return CDense<T>(m_data);
}

template <typename T>
DeviceVar<T>::~DeviceVar()
{
    if(m_data)
        Memory().free(m_data).wait();
}
}  // namespace muda
