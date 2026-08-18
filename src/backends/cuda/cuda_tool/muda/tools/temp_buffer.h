#pragma once
#include <cuda_runtime.h>
#include <muda/check/check.h>
namespace muda::details
{
template <typename T>
class TempBuffer
{
  public:
    TempBuffer() {}

    TempBuffer(size_t size) { resize(size); }

    ~TempBuffer()
    {
        if(m_data)
        {
            // we don't check the error here to prevent exception when app is shutting down
            cudaFree(m_data);
        }
    }

    TempBuffer(TempBuffer&& other) noexcept
    {
        m_size           = other.m_size;
        m_capacity       = other.m_capacity;
        m_data           = other.m_data;
        other.m_size     = 0;
        other.m_capacity = 0;
        other.m_data     = nullptr;
    }

    TempBuffer& operator=(TempBuffer&& other) noexcept
    {
        if(this == &other)
        {
            return *this;
        }
        m_size           = other.m_size;
        m_capacity       = other.m_capacity;
        m_data           = other.m_data;
        other.m_size     = 0;
        other.m_capacity = 0;
        other.m_data     = nullptr;
        return *this;
    }

    // no change on copy
    TempBuffer(const TempBuffer&) noexcept {}
    // no change on copy
    TempBuffer& operator=(const TempBuffer&) noexcept { return *this; }

    void copy_to(std::vector<T>& vec, cudaStream_t stream = nullptr) const
    {
        vec.resize(m_size);
        checkCudaErrors(cudaMemcpyAsync(
            vec.data(), m_data, m_size * sizeof(T), cudaMemcpyDeviceToHost, stream));
    }

    void copy_from(TempBuffer<T>& other, cudaStream_t stream = nullptr)
    {
        resize(other.size());
        checkCudaErrors(cudaMemcpyAsync(
            m_data, other.data(), other.size() * sizeof(T), cudaMemcpyDeviceToDevice, stream));
    }

    void copy_from(const std::vector<T>& vec, cudaStream_t stream = nullptr)
    {
        resize(vec.size());
        checkCudaErrors(cudaMemcpyAsync(
            m_data, vec.data(), vec.size() * sizeof(T), cudaMemcpyHostToDevice, stream));
    }

    TempBuffer(const std::vector<T>& vec) { copy_from(vec); }

    TempBuffer& operator=(const std::vector<T>& vec)
    {
        copy_from(vec);
        return *this;
    }

    void reserve(size_t new_cap, cudaStream_t stream = nullptr)
    {
        if(new_cap <= m_capacity)
        {
            return;
        }
        T* new_data = nullptr;
        checkCudaErrors(cudaMalloc(&new_data, new_cap * sizeof(T)));
        if(m_data)
        {
            checkCudaErrors(cudaFree(m_data));
        }
        m_data     = new_data;
        m_capacity = new_cap;
    }

    void resize(size_t size, cudaStream_t stream = nullptr)
    {
        if(size <= m_capacity)
        {
            m_size = size;
            return;
        }
        reserve(size, stream);
        m_size = size;
    }

    void free() noexcept
    {
        m_size     = 0;
        m_capacity = 0;
        if(m_data)
        {
            checkCudaErrors(cudaFree(m_data));
            m_data = nullptr;
        }
    }

    auto size() const noexcept { return m_size; }
    auto data() const noexcept { return m_data; }
    auto capacity() const noexcept { return m_capacity; }

  private:
    size_t m_size     = 0;
    size_t m_capacity = 0;
    T*     m_data     = nullptr;
};

using ByteTempBuffer = TempBuffer<std::byte>;
}  // namespace muda::details
