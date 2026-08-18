#pragma once
#include <string>
#include <cinttypes>
#include <muda/tools/string_pointer.h>
#include <muda/buffer/device_buffer.h>
#include <muda/ext/field/field_entry_type.h>
#include <muda/ext/field/field_entry_base_data.h>
#include <muda/ext/field/field_entry_view.h>
#include <muda/tools/host_device_config.h>

namespace muda
{
class SubField;
class SubFieldInterface;
template <FieldEntryLayout Layout>
class SubFieldImpl;

class FieldEntryBase
{
    template <FieldEntryLayout layout>
    friend class SubFieldImpl;

  public:
    FieldEntryBase(SubField&            field,
                   FieldEntryLayoutInfo layout_info,
                   FieldEntryType       type,
                   uint2                shape,
                   uint32_t             m_elem_byte_size,
                   std::string_view     name)
        : m_field{field}
        , m_name{name}
    {
        auto& info          = m_core.m_info;
        info.layout_info    = layout_info;
        info.type           = type;
        info.shape          = shape;
        info.elem_byte_size = m_elem_byte_size;

        m_core.m_name =
            m_field.m_field.m_string_cache[std::string{m_field.name()} + "." + m_name];
    }
    ~FieldEntryBase() = default;

  protected:
    friend class SubField;
    friend class SubFieldInterface;
    template <FieldEntryLayout Layout>
    friend class SubFieldImpl;

    virtual void async_copy_to_new_place(HostDeviceConfigView<FieldEntryCore> vfc) const = 0;

    // delete copy
    FieldEntryBase(const FieldEntryBase&)            = delete;
    FieldEntryBase& operator=(const FieldEntryBase&) = delete;

    SubField&   m_field;
    std::string m_name;
    // a parameter struct that can be copy between host and device.
    FieldEntryCore                   m_core;
    HostDeviceConfig<FieldEntryCore> m_host_device_core;

    MUDA_GENERIC const auto& core() const { return m_core; }

  public:
    MUDA_GENERIC auto layout_info() const { return core().layout_info(); }
    MUDA_GENERIC auto layout() const { return core().layout(); }
    MUDA_GENERIC auto count() const { return core().count(); }
    MUDA_GENERIC auto elem_byte_size() const { return core().elem_byte_size(); }
    MUDA_GENERIC auto shape() const { return core().shape(); }
    MUDA_GENERIC auto struct_stride() const { return core().struct_stride(); }
    MUDA_GENERIC auto name() const { return std::string_view{m_name}; }
};

template <typename T, FieldEntryLayout Layout, int M, int N>
class FieldEntry : public FieldEntryBase
{
    static_assert(M > 0 && N > 0, "M and N must be positive");

  public:
    using ValueT = typename FieldEntryView<T, Layout, M, N>::ValueT;

    FieldEntry(SubField& field, FieldEntryLayoutInfo layout, FieldEntryType type, std::string_view name)
        : FieldEntryBase{field,
                         layout,
                         type,
                         make_uint2(static_cast<uint32_t>(M), static_cast<uint32_t>(N)),
                         sizeof(T),
                         name}
    {
    }
    FieldEntry(SubField& field, FieldEntryLayoutInfo layout, FieldEntryType type, uint2 shape, std::string_view name)
        : FieldEntryBase{field, layout, type, shape, sizeof(T), name}
    {
    }

    FieldEntryView<T, Layout, M, N> view()
    {
        MUDA_ASSERT(m_field.data_buffer() != nullptr, "Resize the field before you use it!");
        return FieldEntryView<T, Layout, M, N>{
            m_host_device_core.view(), 0, static_cast<int>(m_core.count())};
    }

    CFieldEntryView<T, Layout, M, N> view() const
    {
        MUDA_ASSERT(m_field.data_buffer() != nullptr, "Resize the field before you use it!");
        return CFieldEntryView<T, Layout, M, N>{
            m_host_device_core.view(), 0, static_cast<int>(m_core.count())};
    }

    auto view(int offset) { return view().subview(offset); }
    auto view(int offset) const { return view().subview(offset); }

    auto view(int offset, int count) { return view().subview(offset, count); }
    auto view(int offset, int count) const
    {
        return view().subview(offset, count);
    }

    FieldEntryViewer<T, Layout, M, N>  viewer() { return view().viewer(); }
    CFieldEntryViewer<T, Layout, M, N> cviewer() const
    {
        return view().viewer();
    }

    void copy_to(DeviceBuffer<ValueT>& dst) const;
    void copy_to(std::vector<ValueT>& dst) const;

    void copy_from(const DeviceBuffer<ValueT>& src);
    void copy_from(const std::vector<ValueT>& src);

    template <FieldEntryLayout SrcLayout>
    void copy_from(const FieldEntry<T, SrcLayout, M, N>& src);

    virtual void async_copy_to_new_place(HostDeviceConfigView<FieldEntryCore> new_place) const override
    {
        using DstView = FieldEntryView<T, Layout, M, N>;
        auto dst = DstView{new_place, 0, static_cast<int>(new_place->count())};

        if(new_place->count() < this->count())  // shrinking
        {
            if(!m_field.allow_inplace_shrink())  // typically SoA don't allow inplace shrinking
            {
                BufferLaunch().resize(m_workpace, new_place->count());
                FieldEntryLaunch().copy(m_workpace.view(),
                                        std::as_const(*this).view(0, new_place->count()));  // copy self to workspace
                FieldEntryLaunch().copy(dst,
                                        m_workpace.view());  // copy workspace to dst
            }
            // else do nothing, trivial shrink
        }
        else if(new_place->count() > this->count())  // expanding
        {
            // safe direct copy
            FieldEntryLaunch().copy(dst.subview(0, this->count()),
                                    std::as_const(*this).view());
        }
        else
        {
            // do thing
        }
    }

    void fill(const ValueT& value);

  private:
    mutable DeviceBuffer<ValueT> m_workpace;  // for data copy, if needed
};
}  // namespace muda

#include "details/field_entry.inl"