#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/debug.h>

namespace uipc::backend::cuda_tool
{
// Dense extents for 2D / 3D buffers (row-major, contiguous).
struct Extent2D
{
    size_t   h = 0, w = 0;
    __host__ __device__ constexpr Extent2D() = default;
    __host__ __device__ constexpr Extent2D(size_t h, size_t w)
        : h(h)
        , w(w)
    {
    }
    __host__ __device__ constexpr size_t count() const { return h * w; }
};

// ---------------------------------------------------------------------------
// Dense1D: C/C++ array like 1D viewer, indexing (x).
// Ported from muda's viewer/dense/dense_1d.h.
// ---------------------------------------------------------------------------
template <bool IsConst, typename T>
class Dense1DT
{
    template <typename U>
    using auto_const_t = std::conditional_t<IsConst, const U, U>;

    template <bool OtherIsConst, typename U>
    friend class Dense1DT;

  public:
    using value_type     = T;
    using ConstViewer    = Dense1DT<true, T>;
    using NonConstViewer = Dense1DT<false, T>;
    using ThisViewer     = Dense1DT<IsConst, T>;

  protected:
    auto_const_t<T>* m_data = nullptr;
    int              m_dim  = 0;

  public:
    __host__ __device__ Dense1DT() noexcept = default;

    __host__ __device__ Dense1DT(auto_const_t<T>* p, int dim) noexcept
        : m_data(p)
        , m_dim(dim)
    {
    }

    // from a contiguous buffer view (muda parity)
    template <bool B = IsConst, std::enable_if_t<!B, int> = 0>
    __host__ __device__ Dense1DT(BufferView<T> v) noexcept
        : m_data(v.data())
        , m_dim((int)v.size())
    {
    }
    __host__ __device__ Dense1DT(CBufferView<T> v) noexcept
        : m_data(v.data())
        , m_dim((int)v.size())
    {
    }

    __host__ __device__ Dense1DT(const Dense1DT& other) = default;

    // non-const -> const conversion
    template <bool OtherIsConst>
    __host__ __device__ Dense1DT(const Dense1DT<OtherIsConst, T>& other) noexcept
        requires(!OtherIsConst)
        : m_data(other.data())
        , m_dim(other.dim())
    {
        static_assert(!OtherIsConst);
    }

    __host__ __device__ ConstViewer as_const() const noexcept
    {
        return ConstViewer{*this};
    }

    __host__ __device__ auto_const_t<T>& operator()(int x) const noexcept
    {
        UIPC_KERNEL_ASSERT(m_data != nullptr, "Dense1D: data is null");
        UIPC_KERNEL_ASSERT(x >= 0 && x < m_dim, "Dense1D: out of range, index=%d, dim=%d", x, m_dim);
        return m_data[x];
    }

    __host__ __device__ auto_const_t<T>* data() const noexcept
    {
        return m_data;
    }

    __host__ __device__ int total_size() const noexcept { return m_dim; }

    __host__ __device__ int dim() const noexcept { return m_dim; }

    __host__ __device__ ThisViewer subview(int offset) const noexcept
    {
        UIPC_KERNEL_ASSERT(offset >= 0 && offset <= m_dim,
                           "Dense1D: subview out of range, offset=%d, dim=%d",
                           offset,
                           m_dim);
        return ThisViewer{m_data + offset, m_dim - offset};
    }

    __host__ __device__ ThisViewer subview(int offset, int size) const noexcept
    {
        UIPC_KERNEL_ASSERT(offset >= 0 && offset + size <= m_dim,
                           "Dense1D: subview out of range, offset=%d, "
                           "size=%d, dim=%d",
                           offset,
                           size,
                           m_dim);
        return ThisViewer{m_data + offset, size};
    }
};

template <typename T>
using Dense1D = Dense1DT<false, T>;
template <typename T>
using CDense1D = Dense1DT<true, T>;

// ---------------------------------------------------------------------------
// Dense2D: 2D array viewer, indexing (x, y):
//   non-pitched: x * dim_y + y;  pitched: (byte*)data + x * pitch + y * sizeof(T)
// y moves faster than x (same as a C/C++ 2D array). Ported from muda's
// viewer/dense/dense_2d.h; argument order of operator() is kept identical.
// ---------------------------------------------------------------------------
template <bool IsConst, typename T>
class Dense2DBase
{
    template <typename U>
    using auto_const_t = std::conditional_t<IsConst, const U, U>;

    template <bool OtherIsConst, typename U>
    friend class Dense2DBase;

  public:
    using value_type     = T;
    using ConstViewer    = Dense2DBase<true, T>;
    using NonConstViewer = Dense2DBase<false, T>;
    using ThisViewer     = Dense2DBase<IsConst, T>;

  protected:
    auto_const_t<T>* m_data        = nullptr;
    int2             m_offset      = make_int2(0, 0);
    int2             m_dim         = make_int2(0, 0);
    int              m_pitch_bytes = 0;

  public:
    __host__ __device__ Dense2DBase() noexcept = default;

    __host__ __device__ Dense2DBase(auto_const_t<T>* p,
                                    const int2&      offset,
                                    const int2&      dim,
                                    int              pitch_bytes) noexcept
        : m_data(p)
        , m_offset(offset)
        , m_dim(dim)
        , m_pitch_bytes(pitch_bytes)
    {
    }

    __host__ __device__ ConstViewer as_const() const noexcept
    {
        return ConstViewer{m_data, m_offset, m_dim, m_pitch_bytes};
    }

    __host__ __device__ auto_const_t<T>& operator()(int x, int y) noexcept
    {
        UIPC_KERNEL_ASSERT(m_data != nullptr, "Dense2D: data is null");
        UIPC_KERNEL_ASSERT(x >= 0 && x < m_dim.x && y >= 0 && y < m_dim.y,
                           "Dense2D: out of range, index=(%d, %d), dim=(%d, %d)",
                           x,
                           y,
                           m_dim.x,
                           m_dim.y);
        x += m_offset.x;
        y += m_offset.y;
        auto* height_begin =
            reinterpret_cast<auto_const_t<std::byte>*>(m_data) + x * m_pitch_bytes;
        return *(reinterpret_cast<auto_const_t<T>*>(height_begin) + y);
    }

    __host__ __device__ auto_const_t<T>& operator()(const int2& xy) noexcept
    {
        return operator()(xy.x, xy.y);
    }

    __host__ __device__ auto_const_t<T>& flatten(int i) noexcept
    {
        UIPC_KERNEL_ASSERT(i >= 0 && i < total_size(),
                           "Dense2D: out of range, index=%d, total_size=%d",
                           i,
                           total_size());
        auto x = i / m_dim.y;
        auto y = i % m_dim.y;
        return operator()(x, y);
    }

    __host__ __device__ auto_const_t<T>* data() noexcept { return m_data; }

    __host__ __device__ const T& operator()(int x, int y) const noexcept
    {
        return const_cast<ThisViewer&>(*this)(x, y);
    }

    __host__ __device__ const T& operator()(const int2& xy) const noexcept
    {
        return const_cast<ThisViewer&>(*this)(xy.x, xy.y);
    }

    __host__ __device__ const T& flatten(int i) const noexcept
    {
        return const_cast<ThisViewer&>(*this).flatten(i);
    }

    __host__ __device__ const T* data() const noexcept { return m_data; }

    __host__ __device__ int total_size() const noexcept
    {
        return m_dim.x * m_dim.y;
    }

    __host__ __device__ int area() const noexcept { return total_size(); }

    __host__ __device__ int2 dim() const noexcept { return m_dim; }

    __host__ __device__ int pitch_bytes() const noexcept
    {
        return m_pitch_bytes;
    }
};

template <typename T>
using Dense2D = Dense2DBase<false, T>;
template <typename T>
using CDense2D = Dense2DBase<true, T>;

// ---------------------------------------------------------------------------
// make functions (muda parity)
// ---------------------------------------------------------------------------
template <typename T>
__host__ __device__ inline auto make_dense_1d(T* data, int dimx) noexcept
{
    return Dense1D<T>(data, dimx);
}

template <typename T, int N>
__host__ __device__ inline auto make_dense_1d(T (&data)[N]) noexcept
{
    return Dense1D<T>(data, N);
}

template <typename T>
__host__ __device__ inline auto make_dense_2d(T* data, const int2& dim) noexcept
{
    return Dense2D<T>{data, make_int2(0, 0), dim, static_cast<int>(dim.y * sizeof(T))};
}

template <typename T>
__host__ __device__ inline auto make_dense_2d(T* data, int dimx, int dimy) noexcept
{
    return make_dense_2d(data, make_int2(dimx, dimy));
}
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
    __host__ __device__ Extent2D extent() const { return m_extent; }
    __host__ __device__ const T& operator()(size_t y, size_t x) const
    {
        return m_data[y * m_extent.w + x];
    }
    // dense-viewer form (muda parity): row-major, first arg is the row
    __host__ __device__ CDense2D<T> cviewer() const
    {
        return CDense2D<T>{m_data,
                           make_int2(0, 0),
                           make_int2((int)m_extent.h, (int)m_extent.w),
                           (int)(m_extent.w * sizeof(T))};
    }
    __host__ __device__ CDense2D<T> viewer() const { return cviewer(); }

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
    // upload from host memory (H2D, stream-ordered), muda parity
    void copy_from(const T* host, cudaStream_t s = default_stream()) const
    {
        size_t n = this->m_extent.count();
        if(n)
            CUDA_TOOL_CHECK(cudaMemcpyAsync(data(), host, n * sizeof(T), cudaMemcpyHostToDevice, s));
    }
    __host__ __device__ CBuffer2DView<T> cview() const { return *this; }
    __host__ __device__ operator CBuffer2DView<T>() const { return *this; }
    __host__ __device__ Dense2D<T> viewer() const
    {
        return Dense2D<T>{const_cast<T*>(this->m_data),
                          make_int2(0, 0),
                          make_int2((int)this->m_extent.h, (int)this->m_extent.w),
                          (int)(this->m_extent.w * sizeof(T))};
    }
    __host__ __device__ CDense2D<T> cviewer() const
    {
        return CDense2D<T>{this->m_data,
                           make_int2(0, 0),
                           make_int2((int)this->m_extent.h, (int)this->m_extent.w),
                           (int)(this->m_extent.w * sizeof(T))};
    }
};
}  // namespace uipc::backend::cuda_tool
