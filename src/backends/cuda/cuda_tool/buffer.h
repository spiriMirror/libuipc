#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/view.h>
#include <cuda_tool/view_nd.h>
#include <cuda_tool/launch.h>
#include <thrust/device_ptr.h>
#include <vector>

namespace uipc::backend::cuda_tool
{
namespace details
{
    // raw template kernels backing BufferLaunch (no lambda machinery)
    template <typename T>
    __global__ void buffer_fill_kernel(BufferView<T> dst, T value)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= (int)dst.size())
            return;
        dst[i] = value;
    }

    template <typename T>
    __global__ void buffer_copy_kernel(BufferView<T> dst, CBufferView<T> src)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= (int)dst.size())
            return;
        dst[i] = src[i];
    }
}  // namespace details

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
    // value assignment uploads to device memory (muda parity)
    DeviceVar& operator=(const T& value)
    {
        copy_from(value);
        return *this;
    }

    T* data() { return m_data; }
    const T* data() const { return m_data; }

    VarView<T>  view() { return VarView<T>{m_data}; }
    CVarView<T> cview() const { return CVarView<T>{m_data}; }
    Dense<T>    viewer() { return Dense<T>{m_data}; }
    CDense<T>   cviewer() const { return CDense<T>{m_data}; }
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

    // implicit host read-back (D2H copy + sync), mirroring cuda_tool::DeviceVar
    operator T() const
    {
        T v{};
        copy_to(v);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(default_stream()));
        return v;
    }

  private:
    T* m_data = nullptr;
};

// A resizable device array (replacement for cuda_tool::DeviceBuffer / DeviceVector).
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

    // thrust-compatible device iterators (muda DeviceVector parity)
    thrust::device_ptr<T>       begin() { return thrust::device_ptr<T>(m_data); }
    thrust::device_ptr<T>       end() { return thrust::device_ptr<T>(m_data + m_size); }
    thrust::device_ptr<const T> begin() const { return thrust::device_ptr<const T>(m_data); }
    thrust::device_ptr<const T> end() const
    {
        return thrust::device_ptr<const T>(m_data + m_size);
    }

    void resize(size_t n, cudaStream_t s = default_stream())
    {
        size_t old = m_size;
        reserve(n, s);
        // muda parity: value-initialize the newly expanded portion
        // (zero for trivially-constructible types, T{} otherwise)
        if(n > old)
        {
            if constexpr(std::is_trivially_constructible_v<T>)
            {
                CUDA_TOOL_CHECK(cudaMemsetAsync(m_data + old, 0, (n - old) * sizeof(T), s));
            }
            else
            {
                static_assert(std::is_constructible_v<T>,
                              "T must be default-constructible for resize()");
                fill(subview(old, n - old), T{}, s);
            }
        }
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
    // const view() returns the read-only view (cuda_tool::DeviceBuffer parity;
    // used via std::as_const(buf).view())
    CBufferView<T> view(size_t offset = 0, size_t count = ~0ull) const
    {
        return cview(offset, count);
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
        int n = (int)v.size();
        details::buffer_fill_kernel<<<(n + default_block_dim - 1) / default_block_dim,
                                      default_block_dim,
                                      0,
                                      s>>>(v, value);
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

    void copy_to(T* host, cudaStream_t s = default_stream()) const
    {
        if(m_size)
        {
            CUDA_TOOL_CHECK(cudaMemcpyAsync(host, m_data, m_size * sizeof(T), cudaMemcpyDeviceToHost, s));
            CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        }
    }
    template <typename Alloc>
    void copy_to(std::vector<T, Alloc>& host, cudaStream_t s = default_stream()) const
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
    void fill(const T& v, cudaStream_t s = default_stream()) { m_buf.fill(v, s); }
    Extent2D extent() const { return m_extent; }
    T*       data() { return m_buf.data(); }
    const T* data() const { return m_buf.data(); }

    Buffer2DView<T>  view() { return Buffer2DView<T>{m_buf.data(), m_extent}; }
    CBuffer2DView<T> cview() const { return CBuffer2DView<T>{m_buf.data(), m_extent}; }
    Dense2D<T>       viewer() { return view().viewer(); }
    CDense2D<T>      cviewer() const { return cview().cviewer(); }
                     operator Buffer2DView<T>() { return view(); }
                     operator CBuffer2DView<T>() const { return cview(); }

  private:
    DeviceVector<T> m_buf;
    Extent2D        m_extent;
};

// Stream-ordered buffer operations (replacement for muda::BufferLaunch).
// All device work is done by the raw template kernels above / cudaMemcpyAsync.
class BufferLaunch
{
  public:
    explicit BufferLaunch(cudaStream_t s = default_stream())
        : m_stream(s)
    {
    }

    // fill
    template <typename T>
    BufferLaunch& fill(BufferView<T> dst, const T& value)
    {
        if(dst.size())
            details::buffer_fill_kernel<<<grid_dim_for((int)dst.size()),
                                          default_block_dim,
                                          0,
                                          m_stream>>>(dst, value);
        return *this;
    }
    template <typename T>
    BufferLaunch& fill(VarView<T> dst, const T& value)
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(
            dst.data(), &value, sizeof(T), cudaMemcpyHostToDevice, m_stream));
        return *this;
    }
    template <typename T>
    BufferLaunch& fill(Buffer2DView<T> dst, const T& value)
    {
        return fill(BufferView<T>{dst.data(), dst.extent().count()}, value);
    }

    // device-to-device copy (same size)
    template <typename T>
    BufferLaunch& copy(BufferView<T> dst, CBufferView<T> src)
    {
        if(dst.size() != src.size())
            throw std::runtime_error{"BufferLaunch::copy size mismatch"};
        if(dst.size())
            details::buffer_copy_kernel<<<grid_dim_for((int)dst.size()),
                                          default_block_dim,
                                          0,
                                          m_stream>>>(dst, src);
        return *this;
    }
    template <typename T>
    BufferLaunch& copy(VarView<T> dst, CVarView<T> src)
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(
            dst.data(), src.data(), sizeof(T), cudaMemcpyDeviceToDevice, m_stream));
        return *this;
    }
    // upload (host -> device)
    template <typename T>
    BufferLaunch& copy(BufferView<T> dst, const T* src)
    {
        if(dst.size())
            CUDA_TOOL_CHECK(cudaMemcpyAsync(dst.data(),
                                            src,
                                            dst.size() * sizeof(T),
                                            cudaMemcpyHostToDevice,
                                            m_stream));
        return *this;
    }
    // download (device -> host)
    template <typename T>
    BufferLaunch& copy(T* dst, CBufferView<T> src)
    {
        if(src.size())
            CUDA_TOOL_CHECK(cudaMemcpyAsync(dst,
                                            src.data(),
                                            src.size() * sizeof(T),
                                            cudaMemcpyDeviceToHost,
                                            m_stream));
        return *this;
    }

    // container size management (forwarded to the owning buffer)
    template <typename T>
    BufferLaunch& resize(DeviceVector<T>& buf, size_t n)
    {
        buf.resize(n, m_stream);
        return *this;
    }
    template <typename T>
    BufferLaunch& resize(DeviceVector<T>& buf, size_t n, const T& val)
    {
        buf.resize(n, val, m_stream);
        return *this;
    }
    template <typename T>
    BufferLaunch& resize(DeviceBuffer2D<T>& buf, Extent2D e)
    {
        buf.resize(e, m_stream);
        return *this;
    }
    template <typename T>
    BufferLaunch& reserve(DeviceVector<T>& buf, size_t cap)
    {
        buf.reserve(cap, m_stream);
        return *this;
    }
    template <typename T>
    BufferLaunch& clear(DeviceVector<T>& buf)
    {
        buf.clear();
        return *this;
    }

    BufferLaunch& wait()
    {
        CUDA_TOOL_CHECK(cudaStreamSynchronize(m_stream));
        return *this;
    }

  private:
    static int grid_dim_for(int n)
    {
        return (n + default_block_dim - 1) / default_block_dim;
    }
    cudaStream_t m_stream;
};

}  // namespace uipc::backend::cuda_tool
