#pragma once
#include <cuda.h>
#include <muda/muda_def.h>
#include <cinttypes>
#undef min
#undef max
namespace muda
{
class Extent2D
{
    size_t m_extent[2];

  public:
    MUDA_GENERIC Extent2D() MUDA_NOEXCEPT : m_extent{0, 0} {}

    MUDA_GENERIC Extent2D(size_t height, size_t width) MUDA_NOEXCEPT
        : m_extent{height, width}
    {
    }

    MUDA_GENERIC size_t height() const MUDA_NOEXCEPT { return m_extent[0]; }
    MUDA_GENERIC size_t width() const MUDA_NOEXCEPT { return m_extent[1]; }

    template <typename T>
    MUDA_GENERIC cudaExtent cuda_extent() const MUDA_NOEXCEPT
    {
        return cudaExtent{width() * sizeof(T), height(), 1};
    }

    MUDA_GENERIC bool valid() const
    {
        return m_extent[0] != ~0ull && m_extent[1] != ~0ull;
    }

    static MUDA_GENERIC Extent2D Zero() MUDA_NOEXCEPT { return Extent2D{0, 0}; }
};


class Extent3D
{
    size_t m_extent[3];

  public:
    MUDA_GENERIC Extent3D() MUDA_NOEXCEPT : m_extent{0, 0, 0} {}
    MUDA_GENERIC Extent3D(size_t depth, size_t height, size_t width) MUDA_NOEXCEPT
        : m_extent{depth, height, width}
    {
    }

    MUDA_GENERIC size_t depth() const MUDA_NOEXCEPT { return m_extent[0]; }
    MUDA_GENERIC size_t height() const MUDA_NOEXCEPT { return m_extent[1]; }
    MUDA_GENERIC size_t width() const MUDA_NOEXCEPT { return m_extent[2]; }

    template <typename T>
    MUDA_GENERIC cudaExtent cuda_extent() const MUDA_NOEXCEPT
    {
        return cudaExtent{width() * sizeof(T), height(), depth()};
    }

    MUDA_GENERIC bool valid() const
    {
        return m_extent[0] != ~0ull && m_extent[1] != ~0ull && m_extent[2] != ~0ull;
    }

    static MUDA_GENERIC Extent3D Zero() MUDA_NOEXCEPT
    {
        return Extent3D{0, 0, 0};
    }
};

class Offset2D
{
    size_t m_offset[2];

  public:
    MUDA_GENERIC Offset2D() MUDA_NOEXCEPT : m_offset{0, 0} {}

    static MUDA_GENERIC Offset2D Zero() MUDA_NOEXCEPT { return Offset2D{0, 0}; }

    MUDA_GENERIC Offset2D(size_t offset_in_height, size_t offset_in_width) MUDA_NOEXCEPT
        : m_offset{offset_in_height, offset_in_width}
    {
    }

    MUDA_GENERIC size_t offset_in_height() const MUDA_NOEXCEPT
    {
        return m_offset[0];
    }

    MUDA_GENERIC size_t offset_in_width() const MUDA_NOEXCEPT
    {
        return m_offset[1];
    }

    template <typename T>
    MUDA_GENERIC cudaPos cuda_pos() const MUDA_NOEXCEPT
    {
        return cudaPos{offset_in_width() * sizeof(T), offset_in_height(), 0};
    }
};

class Offset3D
{
    size_t m_offset[3];

  public:
    MUDA_GENERIC Offset3D() MUDA_NOEXCEPT : m_offset{0, 0, 0} {}

    static MUDA_GENERIC Offset3D Zero() MUDA_NOEXCEPT
    {
        return Offset3D{0, 0, 0};
    }

    MUDA_GENERIC Offset3D(size_t offset_in_depth, size_t offset_in_height, size_t offset_in_width) MUDA_NOEXCEPT
        : m_offset{offset_in_depth, offset_in_height, offset_in_width}
    {
    }

    MUDA_GENERIC size_t offset_in_depth() const MUDA_NOEXCEPT
    {
        return m_offset[0];
    }

    MUDA_GENERIC size_t offset_in_height() const MUDA_NOEXCEPT
    {
        return m_offset[1];
    }

    MUDA_GENERIC size_t offset_in_width() const MUDA_NOEXCEPT
    {
        return m_offset[2];
    }

    template <typename T>
    MUDA_GENERIC cudaPos cuda_pos() const MUDA_NOEXCEPT
    {
        return cudaPos{offset_in_width() * sizeof(T), offset_in_height(), offset_in_depth()};
    }
};

MUDA_INLINE MUDA_GENERIC Extent2D as_extent(const Offset2D& offset) MUDA_NOEXCEPT
{
    return Extent2D{offset.offset_in_height(), offset.offset_in_width()};
}

MUDA_INLINE MUDA_GENERIC Extent3D as_extent(const Offset3D& offset) MUDA_NOEXCEPT
{
    return Extent3D{offset.offset_in_depth(), offset.offset_in_height(), offset.offset_in_width()};
}

MUDA_INLINE MUDA_GENERIC Offset2D as_offset(const Extent2D& extent) MUDA_NOEXCEPT
{
    return Offset2D{extent.height(), extent.width()};
}

MUDA_INLINE MUDA_GENERIC Offset3D as_offset(const Extent3D& extent) MUDA_NOEXCEPT
{
    return Offset3D{extent.depth(), extent.height(), extent.width()};
}

MUDA_INLINE MUDA_GENERIC Offset2D min(const Offset2D& lhs, const Offset2D& rhs)
{
    return Offset2D{std::min(lhs.offset_in_height(), rhs.offset_in_height()),
                    std::min(lhs.offset_in_width(), rhs.offset_in_width())};
}

MUDA_INLINE MUDA_GENERIC Offset3D min(const Offset3D& lhs, const Offset3D& rhs)
{
    return Offset3D{std::min(lhs.offset_in_depth(), rhs.offset_in_depth()),
                    std::min(lhs.offset_in_height(), rhs.offset_in_height()),
                    std::min(lhs.offset_in_width(), rhs.offset_in_width())};
}

MUDA_INLINE MUDA_GENERIC Extent2D max(const Extent2D& lhs, const Extent2D& rhs)
{
    return Extent2D{std::max(lhs.height(), rhs.height()),
                    std::max(lhs.width(), rhs.width())};
}

MUDA_INLINE MUDA_GENERIC Extent3D max(const Extent3D& lhs, const Extent3D& rhs)
{
    return Extent3D{std::max(lhs.depth(), rhs.depth()),
                    std::max(lhs.height(), rhs.height()),
                    std::max(lhs.width(), rhs.width())};
}

#define MUDA_DEFINE_COMPARISON_OPERATOR(op)                                                           \
    MUDA_INLINE MUDA_GENERIC bool operator op(const Extent2D& lhs, const Extent2D& rhs) MUDA_NOEXCEPT \
    {                                                                                                 \
        return (lhs.height() op rhs.height()) && (lhs.width() op rhs.width());                        \
    }                                                                                                 \
    MUDA_INLINE MUDA_GENERIC bool operator op(const Extent3D& lhs, const Extent3D& rhs) MUDA_NOEXCEPT \
    {                                                                                                 \
        return (lhs.depth() op rhs.depth()) && (lhs.height() op rhs.height())                         \
               && (lhs.width() op rhs.width());                                                       \
    }

MUDA_DEFINE_COMPARISON_OPERATOR(<=);
MUDA_DEFINE_COMPARISON_OPERATOR(<);
MUDA_DEFINE_COMPARISON_OPERATOR(==);

#undef MUDA_DEFINE_COMPARISON_OPERATOR


#define MUDA_DEFINE_ARITHMATIC_OPERATOR(op)                                          \
    MUDA_INLINE MUDA_GENERIC Extent2D operator op(const Extent2D& lhs,               \
                                                  const Extent2D& rhs) MUDA_NOEXCEPT \
    {                                                                                \
        return Extent2D{lhs.height() op rhs.height(), lhs.width() op rhs.width()};   \
    }                                                                                \
    MUDA_INLINE MUDA_GENERIC Extent3D operator op(const Extent3D& lhs,               \
                                                  const Extent3D& rhs) MUDA_NOEXCEPT \
    {                                                                                \
        return Extent3D{lhs.depth() op  rhs.depth(),                                 \
                        lhs.height() op rhs.height(),                                \
                        lhs.width() op  rhs.width()};                                 \
    }                                                                                \
    MUDA_INLINE MUDA_GENERIC Offset2D operator op(const Offset2D& lhs,               \
                                                  const Offset2D& rhs) MUDA_NOEXCEPT \
    {                                                                                \
        return Offset2D{lhs.offset_in_height() op rhs.offset_in_height(),            \
                        lhs.offset_in_width() op  rhs.offset_in_width()};             \
    }                                                                                \
    MUDA_INLINE MUDA_GENERIC Offset3D operator op(const Offset3D& lhs,               \
                                                  const Offset3D& rhs) MUDA_NOEXCEPT \
    {                                                                                \
        return Offset3D{lhs.offset_in_depth() op  rhs.offset_in_depth(),             \
                        lhs.offset_in_height() op rhs.offset_in_height(),            \
                        lhs.offset_in_width() op  rhs.offset_in_width()};             \
    }

MUDA_DEFINE_ARITHMATIC_OPERATOR(+);
MUDA_DEFINE_ARITHMATIC_OPERATOR(-);
#undef MUDA_DEFINE_ARITHMATIC_OPERATOR


}  // namespace muda