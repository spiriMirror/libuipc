#pragma once
#include <muda/ext/field/field_entry_type.h>
#include <muda/ext/field/field_entry_layout.h>
namespace muda
{
class FieldEntryBaseData
{
  public:
    // common info
    FieldEntryLayoutInfo layout_info;
    FieldEntryType       type = FieldEntryType::None;
    uint2                shape;
    uint32_t             elem_byte_size = ~0;
    //uint32_t             elem_alignment   = ~0;
    uint32_t elem_count       = ~0;
    uint32_t offset_in_struct = ~0;

    // used by soa
    uint32_t offset_in_base_struct = ~0;
    union
    {
        // used by aos and aosoa
        uint32_t struct_stride = ~0;
        // used by soa
        uint32_t elem_count_based_stride;
    };
};
}  // namespace muda