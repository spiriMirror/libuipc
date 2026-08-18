#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cinttypes>
#include <muda/type_traits/type_modifier.h>
#include <muda/viewer/dense/dense_2d.h>
#include <muda/tools/extent.h>
#include <muda/buffer/buffer_info_accessor.h>
#include <muda/view/view_base.h>
namespace muda
{
template <bool IsConst, typename T>
class Buffer2DViewT : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;

    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<T>;

    friend class BufferLaunch;

    template <bool OtherIsConst, typename U>
    friend class Buffer2DViewT;


  public:
    static_assert(!std::is_const_v<T>, "Ty must be non-const");
    using ConstView = Buffer2DViewT<true, T>;
    using ThisView  = Buffer2DViewT<IsConst, T>;

    using CViewer    = CDense2D<T>;
    using Viewer     = Dense2D<T>;
    using ThisViewer = std::conditional_t<IsConst, CViewer, Viewer>;

  private:
    friend class details::buffer::BufferInfoAccessor;

  protected:
    auto_const_t<T>* m_data          = nullptr;
    size_t           m_pitch_bytes   = ~0;
    size_t           m_origin_width  = 0;
    size_t           m_origin_height = 0;
    Offset2D         m_offset;
    Extent2D         m_extent;

  public:
    MUDA_GENERIC Buffer2DViewT() MUDA_NOEXCEPT = default;

    MUDA_GENERIC Buffer2DViewT(const Buffer2DViewT&) MUDA_NOEXCEPT = default;

    template <bool OtherIsConst>
    MUDA_GENERIC Buffer2DViewT(const Buffer2DViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT
        MUDA_REQUIRES(!OtherIsConst);

    MUDA_GENERIC Buffer2DViewT(auto_const_t<T>* data,
                               size_t           pitch_bytes,
                               size_t           origin_width,
                               size_t           origin_height,
                               const Offset2D&  offset,
                               const Extent2D&  extent) MUDA_NOEXCEPT;

    MUDA_GENERIC Buffer2DViewT(auto_const_t<T>* data,
                               size_t           pitch_bytes,
                               const Offset2D&  offset,
                               const Extent2D&  extent) MUDA_NOEXCEPT;

    MUDA_GENERIC ConstView as_const() const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* data(size_t x, size_t y) const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* data(size_t flatten_i) const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* origin_data() const MUDA_NOEXCEPT;

    MUDA_GENERIC ThisView subview(Offset2D offset, Extent2D extent = {}) const MUDA_NOEXCEPT;

    MUDA_GENERIC Extent2D extent() const MUDA_NOEXCEPT;

    MUDA_GENERIC size_t pitch_bytes() const MUDA_NOEXCEPT;

    MUDA_GENERIC Offset2D offset() const MUDA_NOEXCEPT;

    MUDA_GENERIC size_t total_size() const MUDA_NOEXCEPT;

    MUDA_GENERIC CViewer cviewer() const MUDA_NOEXCEPT;

    MUDA_GENERIC ThisViewer viewer() const MUDA_NOEXCEPT;

    MUDA_HOST void copy_to(T* host) const;

    MUDA_HOST void fill(const T& v) MUDA_REQUIRES(!IsConst);

    MUDA_HOST void copy_from(const Buffer2DViewT<true, T>& other) MUDA_REQUIRES(!IsConst);

    MUDA_HOST void copy_from(const T* host) MUDA_REQUIRES(!IsConst);

  private:
    MUDA_GENERIC cudaPitchedPtr cuda_pitched_ptr() const MUDA_NOEXCEPT;
};

template <typename T>
using Buffer2DView = Buffer2DViewT<false, T>;

template <typename T>
using CBuffer2DView = Buffer2DViewT<true, T>;

template <typename T>
struct read_only_view<Buffer2DView<T>>
{
    using type = CBuffer2DView<T>;
};

template <typename T>
struct read_write_view<CBuffer2DView<T>>
{
    using type = Buffer2DView<T>;
};
}  // namespace muda

#include "details/buffer_2d_view.inl"