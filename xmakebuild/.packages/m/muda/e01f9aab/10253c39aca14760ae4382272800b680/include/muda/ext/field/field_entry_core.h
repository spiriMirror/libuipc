#pragma once
#include <muda/tools/string_pointer.h>
#include <muda/ext/field/field_entry_layout.h>
#include <muda/ext/field/field_entry_base_data.h>

namespace muda
{
template <typename T, FieldEntryLayout Layout, int M, int N>
class FieldEntry;

class FieldEntryBase;

// basic field entry info to pass between different field objects
class FieldEntryCore
{
    friend class FieldEntryBase;
    template <FieldEntryLayout layout>
    friend class SubFieldImpl;
    friend class SubFieldInterface;

  public:
    MUDA_GENERIC FieldEntryCore() {}

    MUDA_GENERIC FieldEntryCore(std::byte*                buffer,
                                const FieldEntryBaseData& info,
                                details::StringPointer    name)
        : m_buffer(const_cast<std::byte*>(buffer))
        , m_info(info)
        , m_name(name)
    {
    }

    MUDA_GENERIC FieldEntryCore(const FieldEntryCore& rhs) = default;

    template <typename T>
    MUDA_GENERIC T& cast(std::byte* data);
    template <typename T>
    MUDA_GENERIC const T& cast(const std::byte* data) const;

    // AOSOA
    MUDA_GENERIC uint32_t aosoa_inner_index(int i) const;
    MUDA_GENERIC std::byte* aosoa_struct_begin(int i) const;
    MUDA_GENERIC std::byte* aosoa_elem_addr(int i) const;
    MUDA_GENERIC std::byte* aosoa_elem_addr(int i, int j) const;
    MUDA_GENERIC std::byte* aosoa_elem_addr(int i, int row_index, int col_index) const;

    // SOA
    MUDA_GENERIC std::byte* soa_elem_addr(int i) const;
    MUDA_GENERIC std::byte* soa_elem_addr(int i, int j) const;
    MUDA_GENERIC std::byte* soa_elem_addr(int i, int row_index, int col_index) const;

    // AOS
    MUDA_GENERIC std::byte* aos_struct_begin(int i) const;
    MUDA_GENERIC std::byte* aos_elem_addr(int i) const;
    MUDA_GENERIC std::byte* aos_elem_addr(int i, int j) const;
    MUDA_GENERIC std::byte* aos_elem_addr(int i, int row_index, int col_index) const;

    // generic access
    template <FieldEntryLayout Layout>
    MUDA_GENERIC std::byte* elem_addr(int i) const;
    template <typename T, FieldEntryLayout Layout>
    MUDA_GENERIC T* data(int i) const
    {
        return reinterpret_cast<T*>(elem_addr<Layout>(i));
    }

    template <FieldEntryLayout Layout>
    MUDA_GENERIC std::byte* elem_addr(int i, int j) const;
    template <typename T, FieldEntryLayout Layout>
    MUDA_GENERIC T* data(int i, int j) const
    {
        return reinterpret_cast<T*>(elem_addr<Layout>(i, j));
    }

    template <FieldEntryLayout Layout>
    MUDA_GENERIC std::byte* elem_addr(int i, int row_index, int col_index) const;
    template <typename T, FieldEntryLayout Layout>
    MUDA_GENERIC T* data(int i, int row_index, int col_index) const
    {
        return reinterpret_cast<T*>(elem_addr<Layout>(i, row_index, col_index));
    }

    MUDA_GENERIC auto layout_info() const { return m_info.layout_info; }
    MUDA_GENERIC auto layout() const { return m_info.layout_info.layout(); }
    MUDA_GENERIC auto count() const { return m_info.elem_count; }
    MUDA_GENERIC auto elem_byte_size() const { return m_info.elem_byte_size; }
    MUDA_GENERIC auto shape() const { return m_info.shape; }
    MUDA_GENERIC auto struct_stride() const { return m_info.struct_stride; }
    MUDA_GENERIC auto name() const { return m_name.auto_select(); }
    MUDA_GENERIC auto name_string_pointer() const { return m_name; }

  private:
    mutable std::byte*     m_buffer = nullptr;
    details::StringPointer m_name;
    FieldEntryBaseData     m_info;
};
}  // namespace muda

#include "details/field_entry_core.inl"
