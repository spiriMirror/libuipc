#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cinttypes>
#include <muda/type_traits/type_modifier.h>
#include <muda/viewer/dense/dense_3d.h>
#include <muda/tools/extent.h>
#include <muda/buffer/buffer_info_accessor.h>
#include <muda/view/view_base.h>
namespace muda
{
template <bool IsConst, typename T>
class Buffer3DViewT : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;

    template <bool OtherIsConst, typename U>
    friend class Buffer3DViewT;

  public:
    static_assert(!std::is_const_v<T>, "Ty must be non-const");
    using ConstView    = Buffer3DViewT<true, T>;
    using NonConstView = Buffer3DViewT<false, T>;
    using ThisView     = Buffer3DViewT<IsConst, T>;
    using OtherView    = Buffer3DViewT<!IsConst, T>;

    using CViewer    = CDense3D<T>;
    using Viewer     = Dense3D<T>;
    using ThisViewer = std::conditional_t<IsConst, CViewer, Viewer>;

  private:
    friend class BufferLaunch;
    friend class details::buffer::BufferInfoAccessor;

    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

  protected:
    auto_const_t<T>* m_data             = nullptr;
    size_t           m_pitch_bytes      = ~0;
    size_t           m_pitch_bytes_area = ~0;
    size_t           m_origin_width     = ~0;
    size_t           m_origin_height    = ~0;

    Offset3D m_offset;
    Extent3D m_extent;

  public:
    MUDA_GENERIC Buffer3DViewT() MUDA_NOEXCEPT = default;

    MUDA_GENERIC Buffer3DViewT(const Buffer3DViewT&) MUDA_NOEXCEPT = default;

    template <bool OtherIsConst>
    MUDA_GENERIC Buffer3DViewT(const Buffer3DViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT;

    MUDA_GENERIC
    Buffer3DViewT(auto_const_t<T>* data,
                  size_t           pitch_bytes,
                  size_t           pitch_bytes_area,
                  size_t           origin_width,
                  size_t           origin_height,
                  const Offset3D&  offset,
                  const Extent3D&  extent) MUDA_NOEXCEPT;

    MUDA_GENERIC Buffer3DViewT(T*              data,
                               size_t          pitch_bytes,
                               size_t          pitch_bytes_area,
                               const Offset3D& offset,
                               const Extent3D& extent) MUDA_NOEXCEPT;


    MUDA_GENERIC ConstView as_const() const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* data(size_t x, size_t y, size_t z) const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* data(size_t flatten_i) const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* origin_data() const MUDA_NOEXCEPT;

    MUDA_GENERIC ThisView subview(Offset3D offset, Extent3D extent = {}) const MUDA_NOEXCEPT;

    MUDA_GENERIC ThisViewer viewer() const MUDA_NOEXCEPT;

    MUDA_GENERIC Extent3D extent() const MUDA_NOEXCEPT;

    MUDA_GENERIC Offset3D offset() const MUDA_NOEXCEPT;

    MUDA_GENERIC size_t pitch_bytes() const MUDA_NOEXCEPT;

    MUDA_GENERIC size_t pitch_bytes_area() const MUDA_NOEXCEPT;

    MUDA_GENERIC size_t total_size() const MUDA_NOEXCEPT;

    MUDA_GENERIC CViewer cviewer() const MUDA_NOEXCEPT;

    MUDA_HOST void fill(const T& v) const MUDA_REQUIRES(!IsConst);

    MUDA_HOST void copy_from(const Buffer3DViewT<true, T>& other) const
        MUDA_REQUIRES(!IsConst);

    MUDA_HOST void copy_from(const T* host) const MUDA_REQUIRES(!IsConst);

    MUDA_HOST void copy_to(T* host) const;

  private:
    MUDA_GENERIC cudaPitchedPtr cuda_pitched_ptr() const MUDA_NOEXCEPT;
};

template <typename T>
using Buffer3DView = Buffer3DViewT<false, T>;

template <typename T>
using CBuffer3DView = Buffer3DViewT<true, T>;

template <typename T>
struct read_only_view<Buffer3DView<T>>
{
    using type = CBuffer3DView<T>;
};

template <typename T>
struct read_write_view<CBuffer3DView<T>>
{
    using type = Buffer3DView<T>;
};
}  // namespace muda

#include "details/buffer_3d_view.inl"