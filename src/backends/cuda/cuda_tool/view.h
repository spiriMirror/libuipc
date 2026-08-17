#pragma once
#include <cuda_tool/stream.h>

namespace uipc::backend::cuda_tool
{
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
    __host__ __device__ const T&     operator[](size_t i) const { return data()[i]; }
    __host__ __device__ CBufferView<T> subview(size_t offset, size_t count = ~0ull) const
    {
        if(count == ~0ull)
            count = m_size - offset;
        return CBufferView<T>{data(), count, offset};
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
    __host__ __device__ BufferView<T> subview(size_t offset, size_t count = ~0ull) const
    {
        if(count == ~0ull)
            count = this->m_size - offset;
        return BufferView<T>{data(), count, offset};
    }
    __host__ __device__ CBufferView<T> cview() const { return *this; }
    __host__ __device__                operator CBufferView<T>() const { return *this; }
    __host__ __device__ auto           begin() const { return data(); }
    __host__ __device__ auto           end() const { return data() + this->m_size; }
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
    __host__ __device__ CVarView<T> cview() const { return *this; }
    __host__ __device__             operator CVarView<T>() const { return *this; }
};

// AsViewer adapters used by parallel_for to turn owning buffers into views.
template <typename T>
auto as_device_view(BufferView<T> v)
{
    return v;
}
template <typename T>
auto as_device_view(CBufferView<T> v)
{
    return v;
}
}  // namespace uipc::backend::cuda_tool
