/*****************************************************************/ /**
 * \file   buffer_view.h
 * \brief  A view interface for any array-like liner memory, which can be 
 * constructed from DeviceBuffer/DeviceVector or any thing that is a array-like
 * liner memory, e.g. raw cuda pointer.
 * 
 * \author MuGdxy
 * \date   January 2024
 *********************************************************************/
#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cinttypes>
#include <muda/type_traits/type_modifier.h>
#include <muda/viewer/dense/dense_1d.h>
#include <muda/buffer/buffer_fwd.h>
#include <muda/view/view_base.h>

namespace muda
{
template <bool IsConst, typename T>
class BufferViewT : public ViewBase<IsConst>
{
    using Base = ViewBase<IsConst>;

    template <bool OtherIsConst, typename U>
    friend class BufferViewT;

  public:
    static_assert(!std::is_const_v<T>, "Ty must be non-const");
    using ConstView = BufferViewT<true, T>;
    using ThisView  = BufferViewT<IsConst, T>;

    using CViewer    = CDense1D<T>;
    using Viewer     = Dense1D<T>;
    using ThisViewer = std::conditional_t<IsConst, CViewer, Viewer>;

    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

  protected:
    auto_const_t<T>* m_data   = nullptr;
    size_t           m_offset = 0;
    size_t           m_size   = 0;

  public:
    MUDA_GENERIC BufferViewT() MUDA_NOEXCEPT = default;

    MUDA_GENERIC BufferViewT(const BufferViewT& other) MUDA_NOEXCEPT = default;

    MUDA_GENERIC BufferViewT(auto_const_t<T>* data, size_t offset, size_t size) MUDA_NOEXCEPT;

    MUDA_GENERIC BufferViewT(auto_const_t<T>* data, size_t size) MUDA_NOEXCEPT;

    template <bool OtherIsConst>
    MUDA_GENERIC BufferViewT(const BufferViewT<OtherIsConst, T>& other) MUDA_NOEXCEPT
        MUDA_REQUIRES(!OtherIsConst);

    MUDA_GENERIC ConstView as_const() const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* data() const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* data(size_t i) const MUDA_NOEXCEPT;

    MUDA_GENERIC auto_const_t<T>* origin_data() const MUDA_NOEXCEPT;

    MUDA_GENERIC ThisView subview(size_t offset, size_t size = ~0) const MUDA_NOEXCEPT;

    MUDA_GENERIC ThisViewer viewer() const MUDA_NOEXCEPT;

    MUDA_GENERIC CViewer cviewer() const MUDA_NOEXCEPT;

    MUDA_GENERIC size_t size() const MUDA_NOEXCEPT { return m_size; }

    MUDA_GENERIC size_t offset() const MUDA_NOEXCEPT { return m_offset; }

    MUDA_GENERIC auto_const_t<T>& operator[](size_t i) const MUDA_NOEXCEPT;

    MUDA_HOST void copy_from(const BufferViewT<true, T>& other) const
        MUDA_REQUIRES(!IsConst);

    MUDA_HOST void fill(const T& value) const MUDA_REQUIRES(!IsConst);

    MUDA_HOST void copy_from(const T* host) const MUDA_REQUIRES(!IsConst);

    MUDA_HOST void copy_to(T* host) const;

    /**********************************************************************************
    * BufferView As Iterator
    ***********************************************************************************/

    // Random Access Iterator Interface
    using value_type        = T;
    using reference         = T&;
    using pointer           = T*;
    using iterator_category = std::random_access_iterator_tag;
    using difference_type   = size_t;

    MUDA_GENERIC ThisView  operator+(int i) const MUDA_NOEXCEPT;
    MUDA_GENERIC reference operator*() const MUDA_NOEXCEPT;
    MUDA_GENERIC auto_const_t<T>& operator[](int i) const MUDA_NOEXCEPT;
};

template <typename T>
using BufferView = BufferViewT<false, T>;

template <typename T>
using CBufferView = BufferViewT<true, T>;

template <typename T>
struct read_only_view<BufferView<T>>
{
    using type = CBufferView<T>;
};

template <typename T>
struct read_write_view<CBufferView<T>>
{
    using type = BufferView<T>;
};
}  // namespace muda

#include "details/buffer_view.inl"