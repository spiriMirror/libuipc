#pragma once
#include <cuda_tool/stream.h>

namespace uipc::backend::cuda_tool
{
// Dense extents for 2D / 3D buffers (row-major, contiguous).
struct Extent2D
{
    size_t h = 0, w = 0;
    __host__ __device__ constexpr Extent2D() = default;
    __host__ __device__ constexpr Extent2D(size_t h, size_t w)
        : h(h)
        , w(w)
    {
    }
    __host__ __device__ constexpr size_t count() const { return h * w; }
};
struct Extent3D
{
    size_t d = 0, h = 0, w = 0;
    __host__ __device__ constexpr Extent3D() = default;
    __host__ __device__ constexpr Extent3D(size_t d, size_t h, size_t w)
        : d(d)
        , h(h)
        , w(w)
    {
    }
    __host__ __device__ constexpr size_t count() const { return d * h * w; }
};

template <typename T>
class CBuffer2DView
{
  public:
    CBuffer2DView() = default;
    __host__ __device__ constexpr CBuffer2DView(const T* data, Extent2D e)
        : m_data(data)
        , m_extent(e)
    {
    }
    __host__ __device__ const T* data() const { return m_data; }
    __host__ __device__ Extent2D  extent() const { return m_extent; }
    __host__ __device__ const T&  operator()(size_t y, size_t x) const
    {
        return m_data[y * m_extent.w + x];
    }

  protected:
    const T* m_data = nullptr;
    Extent2D m_extent;
};

template <typename T>
class Buffer2DView : public CBuffer2DView<T>
{
  public:
    Buffer2DView() = default;
    __host__ __device__ constexpr Buffer2DView(T* data, Extent2D e)
        : CBuffer2DView<T>(data, e)
    {
    }
    __host__ __device__ T* data() const { return const_cast<T*>(this->m_data); }
    __host__ __device__ T& operator()(size_t y, size_t x) const
    {
        return data()[y * this->m_extent.w + x];
    }
    __host__ __device__ CBuffer2DView<T> cview() const { return *this; }
    __host__ __device__ operator CBuffer2DView<T>() const { return *this; }
};

template <typename T>
class CBuffer3DView
{
  public:
    CBuffer3DView() = default;
    __host__ __device__ constexpr CBuffer3DView(const T* data, Extent3D e)
        : m_data(data)
        , m_extent(e)
    {
    }
    __host__ __device__ const T* data() const { return m_data; }
    __host__ __device__ Extent3D  extent() const { return m_extent; }
    __host__ __device__ const T&  operator()(size_t z, size_t y, size_t x) const
    {
        return m_data[(z * m_extent.h + y) * m_extent.w + x];
    }

  protected:
    const T* m_data = nullptr;
    Extent3D m_extent;
};

template <typename T>
class Buffer3DView : public CBuffer3DView<T>
{
  public:
    Buffer3DView() = default;
    __host__ __device__ constexpr Buffer3DView(T* data, Extent3D e)
        : CBuffer3DView<T>(data, e)
    {
    }
    __host__ __device__ T* data() const { return const_cast<T*>(this->m_data); }
    __host__ __device__ T& operator()(size_t z, size_t y, size_t x) const
    {
        return data()[(z * this->m_extent.h + y) * this->m_extent.w + x];
    }
    __host__ __device__ CBuffer3DView<T> cview() const { return *this; }
    __host__ __device__ operator CBuffer3DView<T>() const { return *this; }
};
}  // namespace uipc::backend::cuda_tool
