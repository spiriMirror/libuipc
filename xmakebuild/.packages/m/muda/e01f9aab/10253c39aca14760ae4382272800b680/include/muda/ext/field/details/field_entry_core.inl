#include "muda/type_traits/always.h"
namespace muda
{

template <typename T>
MUDA_INLINE MUDA_GENERIC T& FieldEntryCore::cast(std::byte* data)
{
    return *(reinterpret_cast<T*>(data));
}


template <typename T>
MUDA_INLINE MUDA_GENERIC const T& FieldEntryCore::cast(const std::byte* data) const
{
    return *(reinterpret_cast<const T*>(data));
}


template <FieldEntryLayout Layout>
MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::elem_addr(int i) const
{
    if constexpr(Layout == FieldEntryLayout::RuntimeLayout)
    {
        switch(layout())
        {
            case FieldEntryLayout::AoS:
                return aos_elem_addr(i);
            case FieldEntryLayout::SoA:
                return soa_elem_addr(i);
            case FieldEntryLayout::AoSoA:
                return aosoa_elem_addr(i);
        }
    }
    else
    {
        if constexpr(Layout == FieldEntryLayout::AoS)
        {
            return aos_elem_addr(i);
        }
        else if constexpr(Layout == FieldEntryLayout::SoA)
        {
            return soa_elem_addr(i);
        }
        else if constexpr(Layout == FieldEntryLayout::AoSoA)
        {
            return aosoa_elem_addr(i);
        }
        else
        {
            static_assert("invalid layout");
        }
    }
    MUDA_KERNEL_ERROR_WITH_LOCATION("invalid layout: %d", static_cast<int>(layout()));
    return nullptr;
}


template <FieldEntryLayout Layout>
MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::elem_addr(int i, int j) const
{
    if constexpr(Layout == FieldEntryLayout::RuntimeLayout)
    {
        switch(layout())
        {
            case FieldEntryLayout::AoS:
                return aos_elem_addr(i, j);
            case FieldEntryLayout::SoA:
                return soa_elem_addr(i, j);
            case FieldEntryLayout::AoSoA:
                return aosoa_elem_addr(i, j);
        }
    }
    else
    {
        if constexpr(Layout == FieldEntryLayout::AoS)
        {
            return aos_elem_addr(i, j);
        }
        else if constexpr(Layout == FieldEntryLayout::SoA)
        {
            return soa_elem_addr(i, j);
        }
        else if constexpr(Layout == FieldEntryLayout::AoSoA)
        {
            return aosoa_elem_addr(i, j);
        }
        else
        {
            static_assert("invalid layout");
        }
    }
    MUDA_KERNEL_ERROR_WITH_LOCATION("invalid layout: %d", static_cast<int>(layout()));
    return nullptr;
}


template <FieldEntryLayout Layout>
MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::elem_addr(int i, int j, int col_index) const
{

    if constexpr(Layout == FieldEntryLayout::RuntimeLayout)
    {
        switch(layout())
        {
            case FieldEntryLayout::AoS:
                return aos_elem_addr(i, j, col_index);
            case FieldEntryLayout::SoA:
                return soa_elem_addr(i, j, col_index);
            case FieldEntryLayout::AoSoA:
                return aosoa_elem_addr(i, j, col_index);
        }
    }
    else
    {
        if constexpr(Layout == FieldEntryLayout::AoS)
        {
            return aos_elem_addr(i, j, col_index);
        }
        else if constexpr(Layout == FieldEntryLayout::SoA)
        {
            return soa_elem_addr(i, j, col_index);
        }
        else if constexpr(Layout == FieldEntryLayout::AoSoA)
        {
            return aosoa_elem_addr(i, j, col_index);
        }
        else
        {
            static_assert("invalid layout");
        }
    }
    MUDA_KERNEL_ERROR_WITH_LOCATION("invalid layout: %d", static_cast<int>(layout()));
    return nullptr;
}


MUDA_INLINE MUDA_GENERIC uint32_t FieldEntryCore::aosoa_inner_index(int i) const
{
    return i & (layout_info().innermost_array_size() - 1);
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aosoa_struct_begin(int i) const
{
    auto outer_index = i / layout_info().innermost_array_size();
    return m_buffer + outer_index * struct_stride() + m_info.offset_in_struct;
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aosoa_elem_addr(int i) const
{
    return aosoa_struct_begin(i) + elem_byte_size() * aosoa_inner_index(i);
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aosoa_elem_addr(int i, int comp_j) const
{
    auto innermost_array_size = layout_info().innermost_array_size();
    auto struct_begin         = aosoa_struct_begin(i);
    auto inner_index          = aosoa_inner_index(i);
    return struct_begin
           + elem_byte_size() * (innermost_array_size * comp_j + aosoa_inner_index(i));
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aosoa_elem_addr(int i, int row_index, int col_index) const
{
    auto innermost_array_size = layout_info().innermost_array_size();
    auto struct_begin         = aosoa_struct_begin(i);
    auto inner_index          = aosoa_inner_index(i);
    // column major
    auto j = col_index * shape().x + row_index;
    return struct_begin
           + elem_byte_size() * (innermost_array_size * j + aosoa_inner_index(i));
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::soa_elem_addr(int i) const
{
    return m_buffer + m_info.offset_in_struct + m_info.elem_byte_size * i;
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::soa_elem_addr(int i, int comp_j) const
{
    auto offset = m_info.elem_count_based_stride * comp_j + m_info.elem_byte_size * i;
    return m_buffer + m_info.offset_in_struct + offset;
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::soa_elem_addr(int i, int row_index, int col_index) const
{
    // column major
    auto j = col_index * shape().x + row_index;
    auto offset = m_info.elem_count_based_stride * j + m_info.elem_byte_size * i;
    return m_buffer + m_info.offset_in_struct + offset;
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aos_struct_begin(int i) const
{
    return m_buffer + m_info.struct_stride * i;
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aos_elem_addr(int i) const
{
    return aos_struct_begin(i) + m_info.offset_in_struct;
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aos_elem_addr(int i, int comp_j) const
{
    return aos_struct_begin(i) + m_info.offset_in_struct + m_info.elem_byte_size * comp_j;
}


MUDA_INLINE MUDA_GENERIC std::byte* FieldEntryCore::aos_elem_addr(int i, int row_index, int col_index) const
{
    // column major
    auto j = col_index * shape().x + row_index;
    return aos_struct_begin(i) + m_info.offset_in_struct + m_info.elem_byte_size * j;
}

}  // namespace muda