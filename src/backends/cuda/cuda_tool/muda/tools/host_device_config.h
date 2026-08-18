#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <muda/buffer/device_var.h>
#include <muda/view/view_base.h>

namespace muda
{
template <typename T>
class HostDeviceConfigView : public ViewBase<true>
{
    using Base = ViewBase<true>;
    const T* m_host_data;
    const T* m_device_data;

  public:
    using value_type = T;

    MUDA_GENERIC HostDeviceConfigView(const T* host_data, const T* device_data)
        : m_host_data{host_data}
        , m_device_data{device_data}
    {
    }

    MUDA_GENERIC const T* host_data() const MUDA_NOEXCEPT
    {
        return m_host_data;
    }
    MUDA_GENERIC const T* device_data() const MUDA_NOEXCEPT
    {
        return m_device_data;
    }

    MUDA_GENERIC const T* data() const MUDA_NOEXCEPT
    {
#ifdef __CUDA_ARCH__
        return device_data();
#else
        return host_data();
#endif
    }

    MUDA_GENERIC const T* operator->() const MUDA_NOEXCEPT { return data(); }
    MUDA_GENERIC const T& operator*() const MUDA_NOEXCEPT { return *data(); }
};

template <typename T>
class HostDeviceConfig
{
  private:
    friend class BufferLaunch;
    T            m_host_data;
    DeviceVar<T> m_device_data;

  public:
    using value_type = T;

    HostDeviceConfig() = default;
    HostDeviceConfig(const T& value)
        : m_host_data{value}
        , m_device_data{value}
    {
    }

    HostDeviceConfig(const HostDeviceConfig&)               = default;
    HostDeviceConfig(HostDeviceConfig&&) MUDA_NOEXCEPT      = default;
    HostDeviceConfig& operator=(const HostDeviceConfig<T>&) = default;
    HostDeviceConfig& operator=(HostDeviceConfig<T>&&)      = default;

    HostDeviceConfig& operator=(const T& val)
    {
        m_host_data   = val;
        m_device_data = val;
        return *this;
    }

    const T* host_data() const MUDA_NOEXCEPT { return &m_host_data; }

    T* host_data() MUDA_NOEXCEPT { return &m_host_data; }

    const T* device_data() const MUDA_NOEXCEPT { return m_device_data.data(); }
    auto     buffer_view() MUDA_NOEXCEPT { return m_device_data.view(); }

    auto buffer_view() const MUDA_NOEXCEPT { return m_device_data.view(); }

    auto view() const MUDA_NOEXCEPT
    {
        return HostDeviceConfigView<T>{host_data(), device_data()};
    }
};

}  // namespace muda