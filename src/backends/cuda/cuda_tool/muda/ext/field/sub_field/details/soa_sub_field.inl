namespace muda
{
MUDA_INLINE void SubFieldImpl<FieldEntryLayout::SoA>::build_impl()
{
    auto min_alignment = build_options().min_alignment;
    auto max_alignment = build_options().max_alignment;
    // we use the max alignment as the base array size
    auto base_array_size = max_alignment;
    // e.g. base array size = 4
    // a "Struct" is something like the following, where M/V/S are 3 different entries, has type of matrix/vector/scalar
    //tex:
    // $$
    // \begin{bmatrix}
    // M_{11} & M_{11} & M_{11} & M_{11}\\
    // M_{21} & M_{21} & M_{21} & M_{21}\\
    // M_{12} & M_{12} & M_{12} & M_{12}\\
    // M_{22} & M_{22} & M_{22} & M_{22}\\
    // V_x & V_x & V_x & V_x\\
    // V_y & V_y & V_y & V_y\\
    // V_z & V_z & V_z & V_z\\
    // S   & S   & S   & S \\
    // \end{bmatrix}
    // $$
    uint32_t struct_stride = 0;  // the stride of the "Struct"=> SoA total size
    for(auto& e : m_entries)  // in an entry, the elem type is the same (e.g. float/int/double...)
    {
        // elem type = float/double/int ... or User Type
        auto elem_byte_size = e->elem_byte_size();
        // total elem count in innermost array:
        // scalar=1 vector3 = 3, vector4 = 4, matrix3x3 = 9, matrix4x4 = 16, and so on
        auto elem_count = e->shape().x * e->shape().y;
        struct_stride = align(struct_stride, elem_byte_size, min_alignment, max_alignment);
        auto total_elem_count_in_base_array = e->shape().x * e->shape().y * base_array_size;
        // now struct_stride is the offset of the entry in the "Struct"
        e->m_core.m_info.offset_in_base_struct = struct_stride;
        struct_stride += elem_byte_size * total_elem_count_in_base_array;
    }

    MUDA_ASSERT(struct_stride % base_array_size == 0,
                "m_struct_stride should be multiple of base_array_size");

    m_base_struct_stride = struct_stride;

    m_h_copy_map_buffer.reserve(4 * m_entries.size());
    for(size_t i = 0; i < m_entries.size(); ++i)
    {
        auto& e                        = m_entries[i];
        e->m_core.m_info.struct_stride = m_struct_stride;
        auto btye_in_base_array = e->elem_byte_size() * max_alignment;  // the size of the entry in the base array
        auto first_comp_offset_in_base_struct =
            e->m_core.m_info.offset_in_base_struct;  // the offset of the entry in the base struct
        auto comp_count = e->shape().x * e->shape().y;
        for(int i = 0; i < comp_count; ++i)
        {
            details::SoACopyMap copy_map;
            copy_map.offset_in_base_struct =
                first_comp_offset_in_base_struct + i * btye_in_base_array;
            copy_map.elem_byte_size = e->elem_byte_size();
            m_h_copy_map_buffer.push_back(copy_map);
        }
    }
    // copy to device
    m_copy_map_buffer = m_h_copy_map_buffer;
}

MUDA_INLINE size_t SubFieldImpl<FieldEntryLayout::SoA>::require_total_buffer_byte_size(size_t num_elements)
{
    auto base = m_build_options.max_alignment;
    // auto old_count_of_base = (m_num_elements + base - 1) / base;
    auto new_count_of_base = (num_elements + base - 1) / base;
    // auto rounded_new_count = base * new_count_of_base;
    auto total_bytes = m_base_struct_stride * new_count_of_base;
    return total_bytes;
}

MUDA_INLINE void SubFieldImpl<FieldEntryLayout::SoA>::calculate_new_cores(
    std::byte* byte_buffer, size_t total_bytes, size_t element_count, span<FieldEntryCore> new_cores)
{
    auto base = m_build_options.max_alignment;
    // auto old_count_of_base = (m_num_elements + base - 1) / base;
    auto new_count_of_base = (element_count + base - 1) / base;
    auto rounded_new_count = base * new_count_of_base;

    for(auto& new_core : new_cores)
    {
        new_core.m_info.struct_stride = m_struct_stride;
        new_core.m_info.offset_in_struct =
            new_core.m_info.offset_in_base_struct * new_count_of_base;
        new_core.m_info.elem_count_based_stride =
            new_core.m_info.elem_byte_size * rounded_new_count;
    }
}
}  // namespace muda
