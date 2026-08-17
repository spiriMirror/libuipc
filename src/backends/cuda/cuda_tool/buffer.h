#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/view.h>
#include <cuda_tool/view_nd.h>
#include <cuda_tool/launch.h>
#include <vector>

namespace uipc::backend::cuda_tool
{
// A single element in device memory (replacement for muda::DeviceVar).
template <typename T>
class DeviceVar
{
  public:
    using value_type = T;
    DeviceVar()
    {
        CUDA_TOOL_CHECK(cudaMalloc(&m_data, sizeof(T)));
    }
    explicit DeviceVar(const T& value)
        : DeviceVar()
    {
        copy_from(&value);
    }
    ~DeviceVar()
    {
        if(m_data)
            cudaFree(m_data);
    }
    DeviceVar(DeviceVar&& o) noexcept
        : m_data(o.m_data)
    {
        o.m_data = nullptr;
    }
    DeviceVar& operator=(DeviceVar&& o) noexcept
    {
        if(this != &o)
        {
            if(m_data)
                cudaFree(m_data);
            m_data   = o.m_data;
            o.m_data = nullptr;
        }
        return *this;
    }
    DeviceVar(const DeviceVar&)            = delete;
    DeviceVar& operator=(const DeviceVar&) = delete;

    T* data() { return m_data; }
    const T* data() const { return m_data; }

    VarView<T>  view() { return VarView<T>{m_data}; }
    CVarView<T> cview() const { return CVarView<T>{m_data}; }
    VarView<T>  viewer() { return view(); }
    CVarView<T> cviewer() const { return cview(); }
                 operator VarView<T>() { return view(); }
                 operator CVarView<T>() const { return cview(); }

    void copy_from(const T& value, cudaStream_t s = default_stream())
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(m_data, &value, sizeof(T), cudaMemcpyHostToDevice, s));
    }
    void copy_from(const T* host, cudaStream_t s = default_stream())
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(m_data, host, sizeof(T), cudaMemcpyHostToDevice, s));
    }
    void copy_from(const DeviceVar& o, cudaStream_t s = default_stream())
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(m_data, o.m_data, sizeof(T), cudaMemcpyDeviceToDevice, s));
    }
    void copy_to(T& value, cudaStream_t s = default_stream()) const
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(&value, m_data, sizeof(T), cudaMemcpyDeviceToHost, s));
    }
    void copy_to(T* host, cudaStream_t s = default_stream()) const
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(host, m_data, sizeof(T), cudaMemcpyDeviceToHost, s));
    }

    void fill(const T& value, cudaStream_t s = default_stream()) { copy_from(value, s); }

  private:
    T* m_data = nullptr;
};

// A resizable device array (replacement for muda::DeviceBuffer / DeviceVector).
template <typename T>
class DeviceVector
{
  public:
    using value_type = T;
    DeviceVector() = default;
    explicit DeviceVector(size_t n) { resize(n); }
    explicit DeviceVector(size_t n, const T& init) { resize(n, init); }
    explicit DeviceVector(std::initializer_list<T> il) { copy_from(il.begin(), il.size()); }
    explicit DeviceVector(span<const T> host) { copy_from(host.data(), host.size()); }

    ~DeviceVector() { release(); }
    DeviceVector(DeviceVector&& o) noexcept { move_from(std::move(o)); }
    DeviceVector& operator=(DeviceVector&& o) noexcept
    {
        if(this != &o)
        {
            release();
            move_from(std::move(o));
        }
        return *this;
    }
    DeviceVector(const DeviceVector& o) { copy_from(o); }
    DeviceVector& operator=(const DeviceVector& o)
    {
        if(this != &o)
            copy_from(o);
        return *this;
    }

    size_t size() const { return m_size; }
    size_t capacity() const { return m_capacity; }
    T*     data() { return m_data; }
    const T* data() const { return m_data; }

    void resize(size_t n, cudaStream_t s = default_stream())
    {
        reserve(n, s);
        m_size = n;
    }
    void resize(size_t n, const T& init, cudaStream_t s = default_stream())
    {
        size_t old = m_size;
        resize(n, s);
        if(n > old)
            fill(subview(old, n - old), init, s);
    }
    void reserve(size_t cap, cudaStream_t s = default_stream())
    {
        if(cap <= m_capacity)
            return;
        T* p = nullptr;
        CUDA_TOOL_CHECK(cudaMalloc(&p, cap * sizeof(T)));
        if(m_data && m_size)
            CUDA_TOOL_CHECK(cudaMemcpyAsync(p, m_data, m_size * sizeof(T), cudaMemcpyDeviceToDevice, s));
        if(m_data)
            cudaFree(m_data);
        m_data     = p;
        m_capacity = cap;
    }
    void clear() { m_size = 0; }
    void release()
    {
        if(m_data)
            cudaFree(m_data);
        m_data     = nullptr;
        m_size     = 0;
        m_capacity = 0;
    }

    // views
    BufferView<T> view(size_t offset = 0, size_t count = ~0ull)
    {
        if(count == ~0ull)
            count = m_size - offset;
        return BufferView<T>{m_data, count, offset};
    }
    CBufferView<T> cview(size_t offset = 0, size_t count = ~0ull) const
    {
        if(count == ~0ull)
            count = m_size - offset;
        return CBufferView<T>{m_data, count, offset};
    }
    BufferView<T>  viewer() { return view(); }
    CBufferView<T> cviewer() const { return cview(); }
                   operator BufferView<T>() { return view(); }
                   operator CBufferView<T>() const { return cview(); }
    BufferView<T>  subview(size_t offset, size_t count = ~0ull) { return view(offset, count); }

    // fill
    void fill(const T& value, cudaStream_t s = default_stream()) { fill(view(), value, s); }
    void fill(BufferView<T> v, const T& value, cudaStream_t s = default_stream())
    {
        if(v.size() == 0)
            return;
        parallel_for(
            (int)v.size(),
            [v, value] __device__(int i) mutable { v[i] = value; },
            s);
    }

    // copies
    void copy_from(const DeviceVector& o, cudaStream_t s = default_stream())
    {
        resize(o.m_size, s);
        if(o.m_size)
            CUDA_TOOL_CHECK(cudaMemcpyAsync(
                m_data, o.m_data, m_size * sizeof(T), cudaMemcpyDeviceToDevice, s));
    }
    void copy_from(CBufferView<T> o, cudaStream_t s = default_stream())
    {
        resize(o.size(), s);
        if(o.size())
            CUDA_TOOL_CHECK(cudaMemcpyAsync(
                m_data, o.data(), o.size() * sizeof(T), cudaMemcpyDeviceToDevice, s));
    }
    void copy_from(BufferView<T> o, cudaStream_t s = default_stream()) { copy_from(o.cview(), s); }
    void copy_from(const T* host, size_t n, cudaStream_t s = default_stream())
    {
        resize(n, s);
        if(n)
            CUDA_TOOL_CHECK(cudaMemcpyAsync(m_data, host, n * sizeof(T), cudaMemcpyHostToDevice, s));
    }
    void copy_from(span<const T> host, cudaStream_t s = default_stream())
    {
        copy_from(host.data(), host.size(), s);
    }
    void copy_from(span<T> host, cudaStream_t s = default_stream())
    {
        copy_from(host.data(), host.size(), s);
    }

    void copy_to(T* host, cudaStream_t s = default_stream()) const
    {
        if(m_size)
            CUDA_TOOL_CHECK(cudaMemcpyAsync(host, m_data, m_size * sizeof(T), cudaMemcpyDeviceToHost, s));
    }
    void copy_to(std::vector<T>& host, cudaStream_t s = default_stream()) const
    {
        host.resize(m_size);
        copy_to(host.data(), s);
    }

  private:
    void move_from(DeviceVector&& o)
    {
        m_data       = o.m_data;
        m_size       = o.m_size;
        m_capacity   = o.m_capacity;
        o.m_data     = nullptr;
        o.m_size     = 0;
        o.m_capacity = 0;
    }

    T*     m_data     = nullptr;
    size_t m_size     = 0;
    size_t m_capacity = 0;
};

// muda used DeviceBuffer as a fixed-size array; map it to the same implementation.
template <typename T>
using DeviceBuffer = DeviceVector<T>;

// 2D / 3D dense buffers -----------------------------------------------------
template <typename T>
class DeviceBuffer2D
{
  public:
    DeviceBuffer2D() = default;
    DeviceBuffer2D(Extent2D e) { resize(e); }
    ~DeviceBuffer2D() { release(); }
    DeviceBuffer2D(DeviceBuffer2D&&)            = default;
    DeviceBuffer2D& operator=(DeviceBuffer2D&&) = default;
    DeviceBuffer2D(const DeviceBuffer2D&)            = delete;
    DeviceBuffer2D& operator=(const DeviceBuffer2D&) = delete;

    void resize(Extent2D e, cudaStream_t s = default_stream())
    {
        m_extent = e;
        m_buf.resize(e.count(), s);
    }
    void clear() { m_buf.clear(); }
    void release() { m_buf.release(); }
    Extent2D extent() const { return m_extent; }
    T*       data() { return m_buf.data(); }
    const T* data() const { return m_buf.data(); }

    Buffer2DView<T>  view() { return Buffer2DView<T>{m_buf.data(), m_extent}; }
    CBuffer2DView<T> cview() const { return CBuffer2DView<T>{m_buf.data(), m_extent}; }
    Buffer2DView<T>  viewer() { return view(); }
    CBuffer2DView<T> cviewer() const { return cview(); }
                     operator Buffer2DView<T>() { return view(); }
                     operator CBuffer2DView<T>() const { return cview(); }

  private:
    DeviceVector<T> m_buf;
    Extent2D        m_extent;
};

template <typename T>
class DeviceBuffer3D
{
  public:
    DeviceBuffer3D() = default;
    DeviceBuffer3D(Extent3D e) { resize(e); }
    ~DeviceBuffer3D() { release(); }
    DeviceBuffer3D(DeviceBuffer3D&&)            = default;
    DeviceBuffer3D& operator=(DeviceBuffer3D&&) = default;
    DeviceBuffer3D(const DeviceBuffer3D&)            = delete;
    DeviceBuffer3D& operator=(const DeviceBuffer3D&) = delete;

    void resize(Extent3D e, cudaStream_t s = default_stream())
    {
        m_extent = e;
        m_buf.resize(e.count(), s);
    }
    void clear() { m_buf.clear(); }
    void release() { m_buf.release(); }
    Extent3D extent() const { return m_extent; }
    T*       data() { return m_buf.data(); }
    const T* data() const { return m_buf.data(); }

    Buffer3DView<T>  view() { return Buffer3DView<T>{m_buf.data(), m_extent}; }
    CBuffer3DView<T> cview() const { return CBuffer3DView<T>{m_buf.data(), m_extent}; }
    Buffer3DView<T>  viewer() { return view(); }
    CBuffer3DView<T> cviewer() const { return cview(); }
                     operator Buffer3DView<T>() { return view(); }
                     operator CBuffer3DView<T>() const { return cview(); }

  private:
    DeviceVector<T> m_buf;
    Extent3D        m_extent;
};
}  // namespace uipc::backend::cuda_tool
