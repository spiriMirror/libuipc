#pragma once
#include <cuda_tool/stream.h>
#include <type_traits>

namespace uipc::backend::cuda_tool
{
// Minimal viewer base (replacement for cuda_tool::ViewerBase with MUDA_CHECK_ON=0):
// only provides the const-ness traits; no debug-name payload.
template <bool IsConst_ = false>
class ViewerBase
{
  public:
    constexpr static bool IsConst    = IsConst_;
    constexpr static bool IsNonConst = !IsConst_;

  protected:
    template <typename T>
    using auto_const_t = std::conditional_t<IsConst, const T, T>;
    template <typename T>
    using non_const_enable_t = std::enable_if_t<IsNonConst, T>;

  public:
    // debug-label stubs (muda ViewerBase parity; labels are dropped in cuda_tool)
    __host__ __device__ const char* name() const noexcept { return ""; }
    __host__ __device__ const char* kernel_name() const noexcept { return ""; }
    __host__ __device__ const char* kernel_file() const noexcept { return ""; }
    __host__ __device__ int         kernel_line() const noexcept { return 0; }

  private:
    int m_dummy = 0;  // a dummy member to avoid empty class
};

// Read-only / read-write views over contiguous device memory.
// Lightweight value types that are safe to capture by value into device lambdas.

template <typename T>
class BufferView;  // read-write, T* + count
template <typename T>
class CBufferView;  // read-only, const T* + count
template <typename T>
class VarView;  // read-write, single element
template <typename T>
class CVarView;  // read-only, single element

namespace details
{
    // raw template kernel backing BufferView::fill; defined at the bottom of
    // this header, where BufferView is complete
    template <typename T>
    __global__ void buffer_view_fill_kernel(BufferView<T> dst, T value);
}

template <typename T>
class CBufferView
{
  public:
    using value_type = T;

    CBufferView() = default;
    __host__ __device__ constexpr CBufferView(const T* data, size_t size, size_t offset = 0)
        : m_data(data)
        , m_size(size)
        , m_offset(offset)
    {
    }

    __host__ __device__ const T* data(size_t offset = 0) const
    {
        return m_data + m_offset + offset;
    }
    __host__ __device__ size_t       offset() const { return m_offset; }
    __host__ __device__ size_t       size() const { return m_size; }
    // muda viewer call-shape parity: a buffer view is its own (const) viewer
    __host__ __device__ size_t       total_size() const { return m_size; }
    __host__ __device__ CBufferView<T> cviewer() const { return *this; }
    __host__ __device__ CBufferView<T> viewer() const { return *this; }
    __host__ __device__ CBufferView<T> cview() const { return *this; }
    __host__ __device__ const T&     operator[](size_t i) const { return data()[i]; }
    // parenthesis indexing kept for muda-viewer call-shape parity
    __host__ __device__ const T&     operator()(size_t i) const { return data()[i]; }
    __host__ __device__ CBufferView<T> subview(size_t offset, size_t count = ~0ull) const
    {
        if(count == ~0ull)
            count = m_size - offset;
        return CBufferView<T>{data(), count, offset};
    }
    // download to host memory: async D2H copy on `s` + stream sync.
    // Host-only (matches cuda_tool::BufferView::copy_to semantics).
    void copy_to(T* host, cudaStream_t s = default_stream()) const
    {
        if(m_size)
        {
            CUDA_TOOL_CHECK(cudaMemcpyAsync(
                host, data(), m_size * sizeof(T), cudaMemcpyDeviceToHost, s));
            CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        }
    }
    // read-only iteration
    __host__ __device__ auto cbegin() const { return data(); }
    __host__ __device__ auto cend() const { return data() + m_size; }
    __host__ __device__ auto begin() const { return cbegin(); }
    __host__ __device__ auto end() const { return cend(); }

  protected:
    const T* m_data   = nullptr;
    size_t   m_size   = 0;
    size_t   m_offset = 0;
};

template <typename T>
class BufferView : public CBufferView<T>
{
    using Base = CBufferView<T>;

  public:
    BufferView() = default;
    __host__ __device__ constexpr BufferView(T* data, size_t size, size_t offset = 0)
        : Base(data, size, offset)
    {
    }
    // allow slicing a CBufferView into a read-write view (used by muda callers)
    __host__ __device__ explicit BufferView(const CBufferView<T>& other)
        : Base(const_cast<T*>(other.data()), other.size(), other.offset())
    {
    }

    __host__ __device__ T* data(size_t offset = 0) const
    {
        return const_cast<T*>(Base::data(offset));
    }
    __host__ __device__ T& operator[](size_t i) const { return data()[i]; }
    // parenthesis indexing kept for muda-viewer call-shape parity
    __host__ __device__ T& operator()(size_t i) const { return data()[i]; }
    __host__ __device__ BufferView<T> subview(size_t offset, size_t count = ~0ull) const
    {
        if(count == ~0ull)
            count = this->m_size - offset;
        return BufferView<T>{data(), count, offset};
    }
    __host__ __device__ CBufferView<T> cview() const { return *this; }
    __host__ __device__                operator CBufferView<T>() const { return *this; }
    // muda viewer call-shape parity: a buffer view is its own viewer
    __host__ __device__ BufferView<T>  viewer() const { return *this; }
    __host__ __device__ CBufferView<T> cviewer() const { return *this; }
    __host__ __device__ auto           begin() const { return data(); }
    __host__ __device__ auto           end() const { return data() + this->m_size; }

    // fill every element with `value`: async on stream `s`. Host-only
    // (matches cuda_tool::BufferView::fill semantics).
    void fill(const T& value, cudaStream_t s = default_stream()) const
    {
        if(this->m_size)
        {
            int n    = static_cast<int>(this->m_size);
            int grid = (n + default_block_dim - 1) / default_block_dim;
            details::buffer_view_fill_kernel<T>
                <<<grid, default_block_dim, 0, s>>>(*this, value);
        }
    }
    // device-to-device copy (async on `s`); sizes must match.
    void copy_from(CBufferView<T> src, cudaStream_t s = default_stream()) const
    {
        if(src.size() != this->m_size)
            throw std::runtime_error{"BufferView::copy_from size mismatch"};
        if(this->m_size)
            CUDA_TOOL_CHECK(cudaMemcpyAsync(data(),
                                            src.data(),
                                            this->m_size * sizeof(T),
                                            cudaMemcpyDeviceToDevice,
                                            s));
    }
    // upload from host memory: async H2D copy on `s` + stream sync.
    void copy_from(const T* host, cudaStream_t s = default_stream()) const
    {
        if(this->m_size)
        {
            CUDA_TOOL_CHECK(cudaMemcpyAsync(data(),
                                            host,
                                            this->m_size * sizeof(T),
                                            cudaMemcpyHostToDevice,
                                            s));
            CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
        }
    }
};

// Scalar dense viewers (muda Dense<T>/CDense<T> parity): device-capturable
// single-value accessors. Note the pointer ctor is explicit so that
// `dense = 0` always selects operator=(const T&), never a view copy.
template <typename T>
class CDense
{
  public:
    CDense() = default;
    __host__ __device__ explicit constexpr CDense(const T* data)
        : m_data(data)
    {
    }
    __host__ __device__ const T* data() const { return m_data; }
    __host__ __device__ const T& operator*() const { return *m_data; }
    __host__ __device__ const T* operator->() const { return m_data; }
    __host__ __device__          operator const T&() const { return *m_data; }

  protected:
    const T* m_data = nullptr;
};

template <typename T>
class Dense : public CDense<T>
{
  public:
    Dense() = default;
    __host__ __device__ explicit constexpr Dense(T* data)
        : CDense<T>(data)
    {
    }
    __host__ __device__ T* data() const { return const_cast<T*>(CDense<T>::data()); }
    __host__ __device__ T& operator*() const { return *data(); }
    __host__ __device__ T* operator->() const { return data(); }
    __host__ __device__   operator T&() const { return *data(); }
    // device-side write-through assignment / fill
    __host__ __device__ const Dense& operator=(const T& v) const
    {
        *data() = v;
        return *this;
    }
    __host__ __device__ void fill(const T& v) const { *data() = v; }
};

template <typename T>
class CVarView
{
  public:
    CVarView() = default;
    __host__ __device__ constexpr CVarView(const T* data)
        : m_data(data)
    {
    }
    __host__ __device__ const T* data() const { return m_data; }
    __host__ __device__ const T& operator*() const { return *m_data; }
    __host__ __device__ const T* operator->() const { return m_data; }
    __host__ __device__          operator const T&() const { return *m_data; }
    // dense viewer hooks (muda parity)
    __host__ __device__ CDense<T> cviewer() const { return CDense<T>{m_data}; }
    __host__ __device__ CDense<T> viewer() const { return CDense<T>{m_data}; }
    // download to host (D2H + sync), muda parity
    void copy_to(T* host, cudaStream_t s = default_stream()) const
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(host, m_data, sizeof(T), cudaMemcpyDeviceToHost, s));
        CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
    }

  protected:
    const T* m_data = nullptr;
};

template <typename T>
class VarView : public CVarView<T>
{
    using Base = CVarView<T>;

  public:
    VarView() = default;
    __host__ __device__ constexpr VarView(T* data)
        : Base(data)
    {
    }
    __host__ __device__ explicit VarView(const CVarView<T>& other)
        : Base(const_cast<T*>(other.data()))
    {
    }

    __host__ __device__ T* data() const { return const_cast<T*>(Base::data()); }
    __host__ __device__ T& operator*() const { return *data(); }
    __host__ __device__ T* operator->() const { return data(); }
    __host__ __device__   operator T&() const { return *data(); }
    // dense viewer hooks (muda parity)
    __host__ __device__ Dense<T>     viewer() const { return Dense<T>{data()}; }
    __host__ __device__ CDense<T>    cviewer() const { return CDense<T>{data()}; }
    __host__ __device__ CVarView<T>  cview() const { return *this; }
    __host__ __device__              operator CVarView<T>() const { return *this; }
    // host-side upload (H2D), muda parity
    void fill(const T& value, cudaStream_t s = default_stream()) const
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(data(), &value, sizeof(T), cudaMemcpyHostToDevice, s));
    }
    void copy_from(const T* host, cudaStream_t s = default_stream()) const { fill(*host, s); }
    void copy_from(CVarView<T> o, cudaStream_t s = default_stream()) const
    {
        CUDA_TOOL_CHECK(cudaMemcpyAsync(data(), o.data(), sizeof(T), cudaMemcpyDeviceToDevice, s));
    }
};

namespace details
{
    template <typename T>
    __global__ void buffer_view_fill_kernel(BufferView<T> dst, T value)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= static_cast<int>(dst.size()))
            return;
        dst[i] = value;
    }
}  // namespace details
}  // namespace uipc::backend::cuda_tool
