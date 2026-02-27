#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <muda/buffer/var_view.h>

namespace muda
{
template <typename T>
class DeviceVar
{
  private:
    friend class BufferLaunch;
    T* m_data;

  public:
    using value_type = T;

    DeviceVar();
    DeviceVar(const T& value);

    DeviceVar(const DeviceVar& other);
    DeviceVar(DeviceVar&& other) MUDA_NOEXCEPT;
    DeviceVar& operator=(const DeviceVar<T>& other);
    DeviceVar& operator=(DeviceVar<T>&& other);

    // device transfer
    
    DeviceVar& operator=(CVarView<T> other);
    void       copy_from(CVarView<T> other);

    DeviceVar& operator=(const T& val);  // copy from host
    operator T() const;                  // copy to host

    T*       data() MUDA_NOEXCEPT { return m_data; }
    const T* data() const MUDA_NOEXCEPT { return m_data; }

    VarView<T>  view() MUDA_NOEXCEPT { return VarView<T>{m_data}; };
    CVarView<T> view() const MUDA_NOEXCEPT { return CVarView<T>{m_data}; };

    operator VarView<T>() MUDA_NOEXCEPT { return view(); }
    operator CVarView<T>() const MUDA_NOEXCEPT { return view(); }

    Dense<T>  viewer() MUDA_NOEXCEPT;
    CDense<T> cviewer() const MUDA_NOEXCEPT;

    ~DeviceVar();
};
}  // namespace muda

#include "details/device_var.inl"