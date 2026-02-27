#pragma once
#include <muda/tools/string_pointer.h>
#include <muda/view/view_base.h>
#include <muda/ext/field/field_entry_layout.h>
#include <muda/ext/field/field_entry_base_data.h>
#include <muda/ext/field/matrix_map_info.h>
#include <muda/ext/field/field_entry_core.h>
#include <muda/buffer/buffer_view.h>
#include <muda/ext/field/field_entry_viewer.h>
#include <muda/tools/host_device_config.h>

namespace muda
{
template <bool IsConst, typename T, FieldEntryLayout Layout, int M, int N>
class FieldEntryViewCore : public ViewBase<IsConst>
{
    using Base       = ViewBase<IsConst>;
    using ViewerCore = FieldEntryViewerCore<IsConst, T, Layout, M, N>;
    friend class FieldEntryLaunch;

  public:
    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    using ConstViewer    = CFieldEntryViewer<T, Layout, M, N>;
    using NonConstViewer = FieldEntryViewer<T, Layout, M, N>;
    using ThisViewer = std::conditional_t<IsConst, ConstViewer, NonConstViewer>;

    using MatStride      = typename ViewerCore::MatStride;
    using ConstMatMap    = typename ViewerCore::ConstMatMap;
    using NonConstMatMap = typename ViewerCore::NonConstMatMap;
    using ThisMatMap     = typename ViewerCore::ThisMatMap;

    using ThisView = FieldEntryViewCore<IsConst, T, Layout, M, N>;
    using ValueT = std::conditional_t<M == 1 && N == 1, T, Eigen::Matrix<T, M, N>>;

  protected:
    HostDeviceConfigView<FieldEntryCore> m_core;
    MatStride                            m_stride;
    int                                  m_offset = 0;
    int                                  m_size   = 0;

    MUDA_GENERIC T* data(int i) const
    {
        return m_core->template data<T, Layout>(m_offset + i);
    }

    MUDA_GENERIC T* data(int i, int j) const
    {
        return m_core->template data<T, Layout>(m_offset + i, j);
    }

    MUDA_GENERIC T* data(int i, int row_index, int col_index) const
    {
        return m_core->template data<T, Layout>(m_offset + i, row_index, col_index);
    }

  protected:
    struct AsIterator
    {
    };

    MUDA_GENERIC FieldEntryViewCore(HostDeviceConfigView<FieldEntryCore> core, int offset, int size, AsIterator)
        : m_core{core}
        , m_offset{offset}
        , m_size{size}
    {
        // Note:
        // don't put range check here
        m_stride = details::field::make_stride<T, Layout, M, N>(*m_core);
    }

  public:
    MUDA_GENERIC FieldEntryViewCore() = default;
    MUDA_GENERIC FieldEntryViewCore(HostDeviceConfigView<FieldEntryCore> core, int offset, int size)
        : FieldEntryViewCore(core, offset, size, AsIterator{})
    {
        MUDA_KERNEL_ASSERT(offset >= 0 && size >= 0 && offset + size <= core->count(),
                           "(offset,size) is out of range, offset=%d, size=%d, count=%d",
                           offset,
                           size,
                           core->count());
    }

    MUDA_GENERIC auto layout_info() const { return m_core->layout_info(); }
    MUDA_GENERIC auto layout() const { return layout_info().layout(); }
    MUDA_GENERIC auto offset() const { return m_offset; }
    MUDA_GENERIC auto size() const { return m_size; }
    MUDA_GENERIC auto total_count() const { return m_core->count(); }
    MUDA_GENERIC auto elem_byte_size() const
    {
        return m_core->elem_byte_size();
    }
    MUDA_GENERIC auto shape() const { return m_core->shape(); }
    MUDA_GENERIC auto struct_stride() const { return m_core->struct_stride(); }
    MUDA_GENERIC auto name() const { return m_core->name(); }
    MUDA_GENERIC auto viewer() { return ThisViewer{m_core, offset(), size()}; }
    MUDA_GENERIC auto cviewer() const
    {
        return ConstViewer{m_core, offset(), size()};
    }
};

template <bool IsConst, typename T, FieldEntryLayout Layout, int M, int N>
class FieldEntryViewT : public FieldEntryViewCore<IsConst, T, Layout, M, N>
{
    using Base       = FieldEntryViewCore<IsConst, T, Layout, M, N>;
    using ViewerBase = FieldEntryViewerT<IsConst, T, Layout, M, N>;

    template <typename U>
    using auto_const_t = typename Base::template auto_const_t<U>;

    constexpr static bool IsScalarEntry = (M == 1 && N == 1);
    constexpr static bool IsVectorEntry = (M > 1 && N == 1);
    constexpr static bool IsMatrixEntry = (M > 1 && N > 1);

  public:
    using Base::Base;

    using ThisViewer  = typename Base::ThisViewer;
    using ConstViewer = typename Base::ConstViewer;

    using ConstView    = FieldEntryViewT<true, T, Layout, M, N>;
    using NonConstView = FieldEntryViewT<false, T, Layout, M, N>;

    using ThisView = FieldEntryViewT<IsConst, T, Layout, M, N>;
    using ValueT = std::conditional_t<M == 1 && N == 1, T, Eigen::Matrix<T, M, N>>;

    using ConstMatMap = typename Base::ConstMatMap;
    using ThisMatMap  = typename Base::ThisMatMap;

    MUDA_GENERIC FieldEntryViewT(const FieldEntryViewT& other) MUDA_NOEXCEPT = default;

    template <bool OtherIsConst>
    MUDA_GENERIC FieldEntryViewT(const FieldEntryViewT<OtherIsConst, T, Layout, M, N>& other) MUDA_NOEXCEPT
        MUDA_REQUIRES(!OtherIsConst)
        : Base(other)
    {
        static_assert(!OtherIsConst, "Cannot construct const view from non-const view");
    }

    MUDA_GENERIC auto as_const() const
    {
        return ConstView{this->m_core, Base::offset(), Base::size()};
    }

    MUDA_GENERIC auto_const_t<T>* data(int i) const MUDA_REQUIRES(IsScalarEntry)
    {
        static_assert(IsScalarEntry, "data(i) is only available for scalar entries");
        return Base::data(i);
    }

    MUDA_GENERIC auto_const_t<T>* data(int i, int j) const MUDA_REQUIRES(IsVectorEntry)
    {
        static_assert(IsVectorEntry, "data(i,j) is only available for vector entries");
        return Base::data(i, j);
    }

    MUDA_GENERIC auto_const_t<T>* data(int i, int row_index, int col_index) const
        MUDA_REQUIRES(IsMatrixEntry)
    {
        static_assert(IsMatrixEntry, "data(i,row_index,coll_index) is only available for matrix entries");
        return Base::data(i, row_index, col_index);
    }

    MUDA_GENERIC auto subview(int offset) const
    {
        return ThisView{this->m_core, this->m_offset + offset, this->m_size - offset};
    }

    MUDA_GENERIC auto subview(int offset, int size) const
    {
        return ThisView{this->m_core, this->m_offset + offset, size};
    }

    template <FieldEntryLayout SrcLayout>
    MUDA_HOST void copy_from(const FieldEntryViewT<true, T, SrcLayout, M, N>& src) const
        MUDA_REQUIRES(!IsConst);


    MUDA_HOST void copy_from(const CBufferView<ValueT>& src) const MUDA_REQUIRES(!IsConst);


    MUDA_HOST void copy_to(const BufferView<ValueT>& dst) const;


    MUDA_HOST void fill(const ValueT& value) const MUDA_REQUIRES(!IsConst);

    /**********************************************************************************
    * Entry View As Iterator
    ***********************************************************************************/

    class DummyPointer
    {
        ThisMatMap map;

      public:
        MUDA_GENERIC DummyPointer(ThisMatMap map)
            : map(map)
        {
        }
        MUDA_GENERIC auto operator*() { return map; }
    };

    // Random Access Iterator Interface
    using value_type = ValueT;
    using reference = std::conditional_t<IsScalarEntry, auto_const_t<T>&, ThisMatMap>;
    using pointer = std::conditional_t<IsScalarEntry, auto_const_t<T>*, DummyPointer>;
    using iterator_category = std::random_access_iterator_tag;
    using difference_type   = size_t;

    MUDA_GENERIC ThisView operator+(int i) const
    {
        return ThisView{this->m_core,
                        this->m_offset + i,
                        this->m_size - i,
                        typename Base::AsIterator{}};
    }

    MUDA_GENERIC reference operator*() { return (*this)[0]; }

    MUDA_GENERIC reference operator[](int i) const
    {
        if constexpr(IsScalarEntry)
        {
            return *data(i);
        }
        else if constexpr(IsVectorEntry)
        {
            return ThisMatMap{data(i, 0), this->m_stride};
        }
        else if constexpr(IsMatrixEntry)
        {
            return ThisMatMap{data(i, 0, 0), this->m_stride};
        }
        else
        {
            static_assert("invalid M, N");
        }
    }
};

template <typename T, FieldEntryLayout Layout, int M, int N>
using FieldEntryView = FieldEntryViewT<false, T, Layout, M, N>;

template <typename T, FieldEntryLayout Layout, int M, int N>
using CFieldEntryView = FieldEntryViewT<true, T, Layout, M, N>;
}  // namespace muda

namespace muda
{
template <typename T, FieldEntryLayout Layout, int M, int N>
struct read_only_view<FieldEntryView<T, Layout, M, N>>
{
    using type = CFieldEntryView<T, Layout, M, N>;
};

template <typename T, FieldEntryLayout Layout, int M, int N>
struct read_write_view<CFieldEntryView<T, Layout, M, N>>
{
    using type = FieldEntryView<T, Layout, M, N>;
};
}  // namespace muda

#include "details/field_entry_view.inl"