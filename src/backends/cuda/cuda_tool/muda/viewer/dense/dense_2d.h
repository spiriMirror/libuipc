#pragma once
#include <muda/viewer/viewer_base.h>

namespace muda
{
/*****************************************************************************
 *
 * Dense2D (2D array)
 * indexing (x,y)
 *  1) non-pitched:  x * dim_y + y 
 *  2) pitched:      x * pitch + y
 *
 * Note:
 *  1) y moves faster than x, which is the same as C/C++ 2d array
 *  2) as for CUDA Memory2D, x index into height, y index into width.
 *****************************************************************************/

template <bool IsConst, typename T>
class Dense2DBase : public ViewerBase<IsConst>  // TODO
{
    using Base = ViewerBase<IsConst>;

    MUDA_VIEWER_COMMON_NAME(Dense2DBase);


  protected:
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    auto_const_t<T>* m_data;
    int2             m_offset;
    int2             m_dim;
    int              m_pitch_bytes;

  public:
    using value_type     = T;
    using ConstViewer    = Dense2DBase<true, T>;
    using NonConstViewer = Dense2DBase<false, T>;
    using ThisViewer     = Dense2DBase<IsConst, T>;


    MUDA_GENERIC Dense2DBase() MUDA_NOEXCEPT : m_data(nullptr) {}

    MUDA_GENERIC Dense2DBase(auto_const_t<T>* p, const int2& offset, const int2& dim, int pitch_bytes) MUDA_NOEXCEPT
        : m_data(p),
          m_offset(offset),
          m_dim(dim),
          m_pitch_bytes(pitch_bytes)
    {
    }

    MUDA_GENERIC auto as_const() const MUDA_NOEXCEPT
    {
        return ConstViewer{m_data, m_offset, m_dim, m_pitch_bytes};
    }

    MUDA_GENERIC auto_const_t<T>& operator()(int x, int y) MUDA_NOEXCEPT
    {
        check();
        check_range(x, y);

        x += m_offset.x;
        y += m_offset.y;
        auto height_begin =
            reinterpret_cast<auto_const_t<std::byte>*>(m_data) + x * m_pitch_bytes;
        return *((auto_const_t<T>*)(height_begin) + y);
    }

    MUDA_GENERIC auto_const_t<T>& operator()(const int2& xy) MUDA_NOEXCEPT
    {
        return operator()(xy.x, xy.y);
    }

    MUDA_GENERIC auto_const_t<T>& flatten(int i)
    {
        if constexpr(DEBUG_VIEWER)
        {
            MUDA_KERNEL_ASSERT(i >= 0 && i < total_size(),
                               "Dense2D[%s:%s]: out of range, index=%d, total_size=%d. %s(%d)",
                               this->name(),
                               this->kernel_name(),
                               i,
                               total_size(),
                               this->kernel_file(),
                               this->kernel_line());
        }
        auto x = i / m_dim.y;
        auto y = i % m_dim.y;
        return operator()(x, y);
    }

    MUDA_GENERIC auto_const_t<T>* data() MUDA_NOEXCEPT { return m_data; }


    MUDA_GENERIC const T& operator()(const int2& xy) const MUDA_NOEXCEPT
    {
        return remove_const(*this)(xy);
    }


    MUDA_GENERIC const T& operator()(int x, int y) const MUDA_NOEXCEPT
    {
        return remove_const(*this)(x, y);
    }

    MUDA_GENERIC const T& flatten(int i) const
    {
        return remove_const(*this).flatten(i);
    }

    MUDA_GENERIC const T* data() const MUDA_NOEXCEPT { return m_data; }

    MUDA_GENERIC auto total_size() const MUDA_NOEXCEPT
    {
        return m_dim.x * m_dim.y;
    }

    MUDA_GENERIC auto area() const MUDA_NOEXCEPT { return total_size(); }

    MUDA_GENERIC auto dim() const MUDA_NOEXCEPT { return m_dim; }

    MUDA_GENERIC auto pitch_bytes() const MUDA_NOEXCEPT
    {
        return m_pitch_bytes;
    }

  protected:
    MUDA_INLINE MUDA_GENERIC void check_range(int x, int y) const MUDA_NOEXCEPT
    {
        if constexpr(DEBUG_VIEWER)
            if(!(x >= 0 && x < m_dim.x && y >= 0 && y < m_dim.y))
            {
                MUDA_KERNEL_ERROR("Dense2D[%s:%s]: out of range, index=(%d,%d) dim=(%d,%d). %s(%d)",
                                  this->name(),
                                  this->kernel_name(),
                                  x,
                                  y,
                                  m_dim.x,
                                  m_dim.y,
                                  this->kernel_file(),
                                  this->kernel_line());
            }
    }

    MUDA_INLINE MUDA_GENERIC void check() const MUDA_NOEXCEPT
    {
        if constexpr(DEBUG_VIEWER)
        {
            MUDA_KERNEL_ASSERT(m_data,
                               "Dense2D[%s:%s]: m_data is null. %s(%d)",
                               this->name(),
                               this->kernel_name(),
                               this->kernel_file(),
                               this->kernel_line());
        }
    }
};

template <typename T>
using Dense2D = Dense2DBase<false, T>;

template <typename T>
using CDense2D = Dense2DBase<true, T>;


// viewer traits
template <typename T>
struct read_only_view<Dense2D<T>>
{
    using type = CDense2D<T>;
};

template <typename T>
struct read_write_view<CDense2D<T>>
{
    using type = Dense2D<T>;
};

// make functions
template <typename T>
MUDA_INLINE MUDA_GENERIC auto make_cdense_2d(const T* data, const int2& dim) MUDA_NOEXCEPT
{
    return CDense2D<T>{data, make_int2(0, 0), dim, static_cast<int>(dim.y * sizeof(T))};
}

template <typename T>
MUDA_INLINE MUDA_GENERIC auto make_dense_2d(T* data, const int2& dim) MUDA_NOEXCEPT
{
    return Dense2D<T>{data, make_int2(0, 0), dim, static_cast<int>(dim.y * sizeof(T))};
}

template <typename T>
MUDA_INLINE MUDA_GENERIC auto make_cdense_2d(const T* data, int dimx, int dimy) MUDA_NOEXCEPT
{
    return make_cdense_2d(data, make_int2(dimx, dimy));
}

template <typename T>
MUDA_INLINE MUDA_GENERIC auto make_dense_2d(T* data, int dimx, int dimy) MUDA_NOEXCEPT
{
    return make_dense_2d(data, make_int2(dimx, dimy));
}


}  // namespace muda