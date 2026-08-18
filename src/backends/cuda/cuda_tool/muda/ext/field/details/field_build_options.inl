#pragma once
#include <muda/ext/field/field_entry_layout.h>
namespace muda
{
class FieldBuildOptions
{
  public:
    FieldEntryLayoutInfo layout    = FieldEntryLayoutInfo::None;
    uint32_t alignment = 4;  // bytes
};
}