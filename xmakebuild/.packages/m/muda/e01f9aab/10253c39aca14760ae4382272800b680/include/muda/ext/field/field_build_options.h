#pragma once
#include <muda/ext/field/field_entry_layout.h>
namespace muda
{
class FieldBuildOptions
{
  public:
    uint32_t min_alignment = sizeof(int);  // bytes
    uint32_t max_alignment = sizeof(std::max_align_t);
};
}  // namespace muda