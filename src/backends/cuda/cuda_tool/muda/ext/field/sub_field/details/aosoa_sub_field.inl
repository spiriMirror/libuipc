namespace muda
{
MUDA_INLINE void SubFieldImpl<FieldEntryLayout::AoSoA>::build_impl()
{
    auto min_alignment = build_options().min_alignment;
    auto max_alignment = build_options().max_alignment;
    // eg: innermost array size = 4
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
    uint32_t struct_stride = 0;  // the stride of the "Struct"
    for(auto& e : m_entries)  // in an entry, the elem type is the same (e.g. float/int/double...)
    {
        // elem type = float/double/int ... or User Type
        auto elem_byte_size = e->elem_byte_size();
        // innermost array size: most of time the size = 32 (warp size)
        auto inner_array_size = e->layout_info().innermost_array_size();
        // total elem count in innermost array:
        // scalar=1 vector3 = 3, vector4 = 4, matrix3x3 = 9, matrix4x4 = 16, and so on

        struct_stride = align(struct_stride, elem_byte_size, min_alignment, max_alignment);
        // now struct_stride is the offset of the entry in the "Struct"
        e->m_core.m_info.offset_in_struct = struct_stride;

        auto total_elem_count_in_innermost_array = e->shape().x * e->shape().y * inner_array_size;
        struct_stride += elem_byte_size * total_elem_count_in_innermost_array;
    }
    // the final stride of the "Struct" >= struct size
    m_struct_stride = align(struct_stride, struct_stride, min_alignment, max_alignment);

    for(auto& e : m_entries)
    {
        e->m_core.m_info.struct_stride = m_struct_stride;
    }
}

MUDA_INLINE size_t SubFieldImpl<FieldEntryLayout::AoSoA>::require_total_buffer_byte_size(size_t element_count)
{
    size_t outer_size = round_up(element_count, m_layout_info.innermost_array_size());
    return outer_size * m_struct_stride;
}
}  // namespace muda
